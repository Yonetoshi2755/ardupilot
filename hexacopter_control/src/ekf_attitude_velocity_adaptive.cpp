#include "ekf_attitude_velocity_adaptive.h"
#include <cmath>

EKF_AttitudeVelocityAdaptive::EKF_AttitudeVelocityAdaptive() :
    EKF_AttitudeVelocity(),
    _sysid(nullptr),
    _inertia_uncertainty_scale(1.0f),
    _thrust_uncertainty_scale(1.0f)
{
}

void EKF_AttitudeVelocityAdaptive::predict_with_controls(const Vector3f& gyro, 
                                                         const Vector3f& accel,
                                                         const float motor_commands[6], 
                                                         float dt)
{
    if (_sysid == nullptr) {
        // Fall back to standard prediction
        update_imu(gyro, accel, dt);
        return;
    }
    
    // Get identified parameters
    Matrix3f inertia = _sysid->get_inertia_matrix();
    Vector3f damping = _sysid->get_damping_coefficients();
    float thrust_coeff = _sysid->get_thrust_coefficient();
    float torque_coeff = _sysid->get_torque_coefficient();
    
    // State extraction
    Quaternion q(_state[0], _state[1], _state[2], _state[3]);
    Vector3f omega(_state[4], _state[5], _state[6]);
    Vector3f pos(_state[7], _state[8], _state[9]);
    Vector3f vel(_state[10], _state[11], _state[12]);
    Vector3f bias(_state[13], _state[14], _state[15]);
    
    // Compensate gyro with bias
    Vector3f omega_corrected = gyro - bias;
    
    // Compute expected angular acceleration using identified inertia
    Vector3f expected_torque = compute_torque_rates(motor_commands);
    Vector3f angular_accel = inertia.inverse() * (expected_torque - omega % (inertia * omega) - damping * omega);
    
    // Update angular velocity with model-based prediction
    _state[4] = omega.x + angular_accel.x * dt;
    _state[5] = omega.y + angular_accel.y * dt;
    _state[6] = omega.z + angular_accel.z * dt;
    
    // Quaternion update using corrected rates
    float omega_norm = omega_corrected.length();
    if (omega_norm > 0.0001f) {
        float half_angle = 0.5f * omega_norm * dt;
        float cos_half = cosf(half_angle);
        float sin_half = sinf(half_angle) / omega_norm;
        
        Quaternion q_delta(cos_half,
                          sin_half * omega_corrected.x,
                          sin_half * omega_corrected.y,
                          sin_half * omega_corrected.z);
        
        q = quaternion_multiply(q, q_delta);
    }
    
    // Position and velocity update with thrust model
    pos += vel * dt;
    
    // Get rotation matrix
    Matrix3f dcm;
    q.rotation_matrix(dcm);
    
    // Compute thrust acceleration from motors
    Vector3f thrust_accel_body = compute_thrust_acceleration(motor_commands);
    Vector3f thrust_accel_ned = dcm * thrust_accel_body;
    
    // Total acceleration including gravity and drag
    Vector3f total_accel = thrust_accel_ned + Vector3f(0, 0, GRAVITY_MSS);
    total_accel -= damping * vel; // Aerodynamic drag
    
    vel += total_accel * dt;
    
    // Store updated state
    _state[0] = q.q1;
    _state[1] = q.q2;
    _state[2] = q.q3;
    _state[3] = q.q4;
    _state[7] = pos.x;
    _state[8] = pos.y;
    _state[9] = pos.z;
    _state[10] = vel.x;
    _state[11] = vel.y;
    _state[12] = vel.z;
    
    normalize_quaternion();
    
    // Update covariance with adaptive noise
    update_adaptive_noise();
    
    // Modified F matrix computation using identified parameters
    float F[STATE_SIZE][STATE_SIZE];
    compute_F_jacobian_adaptive(F, _state, gyro, accel, motor_commands, dt);
    
    // Covariance propagation
    propagate_covariance(F, dt);
}

Vector3f EKF_AttitudeVelocityAdaptive::compute_thrust_acceleration(const float motors[6]) const
{
    if (_sysid == nullptr) {
        return Vector3f(0, 0, 0);
    }
    
    float thrust_coeff = _sysid->get_thrust_coefficient();
    float mass = _sysid->get_vehicle_mass();
    float total_thrust = 0;
    
    // Sum thrust from all motors
    for (int i = 0; i < 6; i++) {
        total_thrust += thrust_coeff * motors[i] * motors[i];
    }
    
    // Thrust is along body Z axis (down)
    return Vector3f(0, 0, -total_thrust / mass);
}

Vector3f EKF_AttitudeVelocityAdaptive::compute_torque_rates(const float motors[6]) const
{
    if (_sysid == nullptr) {
        return Vector3f(0, 0, 0);
    }
    
    float thrust_coeff = _sysid->get_thrust_coefficient();
    float torque_coeff = _sysid->get_torque_coefficient();
    
    // Motor configuration for X hexacopter
    const float arm_length = 0.25f;
    const float motor_angles[6] = {
        radians(60.0f), radians(120.0f), radians(180.0f),
        radians(240.0f), radians(300.0f), radians(0.0f)
    };
    const float yaw_direction[6] = {1, -1, 1, -1, 1, -1};
    
    Vector3f total_torque(0, 0, 0);
    
    for (int i = 0; i < 6; i++) {
        float thrust = thrust_coeff * motors[i] * motors[i];
        
        // Torque from thrust differential
        total_torque.x += -arm_length * thrust * sinf(motor_angles[i]);
        total_torque.y += arm_length * thrust * cosf(motor_angles[i]);
        
        // Yaw torque from prop rotation
        total_torque.z += torque_coeff * motors[i] * motors[i] * yaw_direction[i];
    }
    
    return total_torque;
}

void EKF_AttitudeVelocityAdaptive::update_adaptive_noise()
{
    if (_sysid == nullptr || !_sysid->is_running()) {
        return;
    }
    
    // During system ID, increase process noise to account for uncertainty
    _inertia_uncertainty_scale = 2.0f;
    _thrust_uncertainty_scale = 2.0f;
    
    // Gradually reduce uncertainty as we get more samples
    float sample_ratio = _sysid->get_sample_count() / 1000.0f;
    sample_ratio = constrain_float(sample_ratio, 0.0f, 1.0f);
    
    _inertia_uncertainty_scale = 2.0f - sample_ratio; // 2.0 -> 1.0
    _thrust_uncertainty_scale = 2.0f - sample_ratio;   // 2.0 -> 1.0
    
    // Apply to process noise
    _gyro_noise_variance *= _inertia_uncertainty_scale;
    _accel_noise_variance *= _thrust_uncertainty_scale;
}

void EKF_AttitudeVelocityAdaptive::compute_F_jacobian_adaptive(float F[STATE_SIZE][STATE_SIZE],
                                                               const float* x,
                                                               const Vector3f& gyro,
                                                               const Vector3f& accel,
                                                               const float motors[6],
                                                               float dt) const
{
    // Initialize F as identity
    memset(F, 0, sizeof(float) * STATE_SIZE * STATE_SIZE);
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        F[i][i] = 1.0f;
    }
    
    if (_sysid == nullptr) {
        // Use standard Jacobian
        compute_F_jacobian(F, x, gyro, accel, dt);
        return;
    }
    
    // Get identified parameters
    Matrix3f inertia = _sysid->get_inertia_matrix();
    Vector3f damping = _sysid->get_damping_coefficients();
    
    // Extract states
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float wx = x[4], wy = x[5], wz = x[6];
    float vx = x[10], vy = x[11], vz = x[12];
    
    // Quaternion dynamics with inertia coupling
    float half_dt = 0.5f * dt;
    
    // dq/dq
    F[0][1] = -half_dt * wx;
    F[0][2] = -half_dt * wy;
    F[0][3] = -half_dt * wz;
    
    F[1][0] = half_dt * wx;
    F[1][2] = half_dt * wz;
    F[1][3] = -half_dt * wy;
    
    F[2][0] = half_dt * wy;
    F[2][1] = -half_dt * wz;
    F[2][3] = half_dt * wx;
    
    F[3][0] = half_dt * wz;
    F[3][1] = half_dt * wy;
    F[3][2] = -half_dt * wx;
    
    // dq/domega
    F[0][4] = -half_dt * q1;
    F[0][5] = -half_dt * q2;
    F[0][6] = -half_dt * q3;
    
    F[1][4] = half_dt * q0;
    F[1][5] = -half_dt * q3;
    F[1][6] = half_dt * q2;
    
    F[2][4] = half_dt * q3;
    F[2][5] = half_dt * q0;
    F[2][6] = -half_dt * q1;
    
    F[3][4] = -half_dt * q2;
    F[3][5] = half_dt * q1;
    F[3][6] = half_dt * q0;
    
    // Angular velocity dynamics with identified inertia
    // domega/domega includes inertia coupling and damping
    Matrix3f inertia_inv = inertia.inverse();
    
    F[4][4] = 1.0f - dt * (damping.x / inertia.a.x);
    F[5][5] = 1.0f - dt * (damping.y / inertia.b.y);
    F[6][6] = 1.0f - dt * (damping.z / inertia.c.z);
    
    // Cross-coupling terms from omega x (I * omega)
    F[4][5] = dt * inertia_inv.a.x * (inertia.c.z - inertia.b.y) * wz;
    F[4][6] = dt * inertia_inv.a.x * (inertia.b.y - inertia.c.z) * wy;
    
    F[5][4] = dt * inertia_inv.b.y * (inertia.a.x - inertia.c.z) * wz;
    F[5][6] = dt * inertia_inv.b.y * (inertia.c.z - inertia.a.x) * wx;
    
    F[6][4] = dt * inertia_inv.c.z * (inertia.b.y - inertia.a.x) * wy;
    F[6][5] = dt * inertia_inv.c.z * (inertia.a.x - inertia.b.y) * wx;
    
    // Position dynamics
    F[7][10] = dt;
    F[8][11] = dt;
    F[9][12] = dt;
    
    // Velocity dynamics with drag
    F[10][10] = 1.0f - dt * damping.x;
    F[11][11] = 1.0f - dt * damping.y;
    F[12][12] = 1.0f - dt * damping.z;
    
    // Velocity coupling to attitude (thrust direction)
    // This requires the thrust acceleration computation
    Vector3f thrust_body = compute_thrust_acceleration(motors);
    
    // dv/dq (velocity change due to attitude change)
    // Simplified - full implementation would compute d(R*thrust)/dq
    float thrust_mag = thrust_body.length();
    if (thrust_mag > 0.1f) {
        // Partial derivatives of rotated thrust with respect to quaternion
        F[10][0] = dt * 2.0f * thrust_mag * (q0*q2 + q1*q3);
        F[10][1] = dt * 2.0f * thrust_mag * (q1*q2 - q0*q3);
        F[10][2] = dt * 2.0f * thrust_mag * (q0*q0 + q1*q1 - 0.5f);
        F[10][3] = dt * 2.0f * thrust_mag * (-q0*q1 + q2*q3);
        
        // Similar for Y and Z velocity components...
    }
}

void EKF_AttitudeVelocityAdaptive::propagate_covariance(const float F[STATE_SIZE][STATE_SIZE], float dt)
{
    // P = F * P * F' + Q
    float P_new[(STATE_SIZE * (STATE_SIZE + 1)) / 2];
    memset(P_new, 0, sizeof(P_new));
    
    // P_new = F * P * F'
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = i; j < STATE_SIZE; j++) {
            float sum = 0;
            for (uint8_t k = 0; k < STATE_SIZE; k++) {
                for (uint8_t l = 0; l < STATE_SIZE; l++) {
                    sum += F[i][k] * P(k, l) * F[j][l];
                }
            }
            P_new[P_index(i, j)] = sum;
        }
    }
    
    // Add adaptive process noise
    P_new[P_index(4, 4)] += _gyro_noise_variance * _inertia_uncertainty_scale * dt;
    P_new[P_index(5, 5)] += _gyro_noise_variance * _inertia_uncertainty_scale * dt;
    P_new[P_index(6, 6)] += _gyro_noise_variance * _inertia_uncertainty_scale * dt;
    
    P_new[P_index(10, 10)] += _accel_noise_variance * _thrust_uncertainty_scale * dt;
    P_new[P_index(11, 11)] += _accel_noise_variance * _thrust_uncertainty_scale * dt;
    P_new[P_index(12, 12)] += _accel_noise_variance * _thrust_uncertainty_scale * dt;
    
    P_new[P_index(13, 13)] += _bias_noise_variance * dt;
    P_new[P_index(14, 14)] += _bias_noise_variance * dt;
    P_new[P_index(15, 15)] += _bias_noise_variance * dt;
    
    // Copy back
    memcpy(_P, P_new, sizeof(_P));
}