#include "ekf_attitude_velocity.h"
#ifdef ARDUPILOT_BUILD
#include <AP_Math/AP_Math.h>
#endif

EKF_AttitudeVelocity::EKF_AttitudeVelocity() :
    _gyro_noise_variance(0.001f),
    _accel_noise_variance(0.1f),
    _bias_noise_variance(0.0001f),
    _gyro_meas_variance(0.001f),
    _accel_meas_variance(0.1f),
    _mag_meas_variance(0.01f),
    _gps_pos_variance(5.0f),
    _gps_vel_variance(0.1f),
    _gravity_ref(0, 0, GRAVITY_MSS),
    _mag_earth(0.2f, 0.0f, 0.4f)  // Typical mid-latitude magnetic field
{
    // Initialize state to zero
    memset(_state, 0, sizeof(_state));
    
    // Initialize quaternion to identity
    _state[QUAT_IDX] = 1.0f;
    
    // Initialize covariance matrix
    memset(_P, 0, sizeof(_P));
    
    // Set initial covariance
    P(0, 0) = P(1, 1) = P(2, 2) = P(3, 3) = 0.01f;  // Quaternion
    P(4, 4) = P(5, 5) = P(6, 6) = 0.1f;   // Angular velocity
    P(7, 7) = P(8, 8) = P(9, 9) = 100.0f; // Position
    P(10, 10) = P(11, 11) = P(12, 12) = 1.0f; // Velocity
    P(13, 13) = P(14, 14) = P(15, 15) = 0.01f; // Gyro bias
}

void EKF_AttitudeVelocity::init(const Vector3f& init_position, const Quaternion& init_attitude)
{
    // Set initial attitude
    _state[QUAT_IDX] = init_attitude.q1;
    _state[QUAT_IDX + 1] = init_attitude.q2;
    _state[QUAT_IDX + 2] = init_attitude.q3;
    _state[QUAT_IDX + 3] = init_attitude.q4;
    normalize_quaternion();
    
    // Set initial position
    _state[POS_IDX] = init_position.x;
    _state[POS_IDX + 1] = init_position.y;
    _state[POS_IDX + 2] = init_position.z;
    
    // Angular velocity and velocity start at zero
    for (int i = OMEGA_IDX; i < OMEGA_IDX + 3; i++) {
        _state[i] = 0;
    }
    for (int i = VEL_IDX; i < VEL_IDX + 3; i++) {
        _state[i] = 0;
    }
    
    // Gyro bias starts at zero
    for (int i = BIAS_IDX; i < BIAS_IDX + 3; i++) {
        _state[i] = 0;
    }
}

void EKF_AttitudeVelocity::predict(float dt)
{
    // Time update without measurements
    // This would typically use the last known angular rates and accelerations
    Vector3f gyro(_state[OMEGA_IDX], _state[OMEGA_IDX + 1], _state[OMEGA_IDX + 2]);
    Vector3f accel(0, 0, -GRAVITY_MSS); // Assume hovering
    
    predict_state(dt);
    
    // Compute state transition Jacobian
    float F[STATE_SIZE][STATE_SIZE];
    compute_F_jacobian(F, _state, gyro, accel, dt);
    
    // Update covariance: P = F * P * F' + Q
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
    
    // Add process noise
    P_new[P_index(4, 4)] += _gyro_noise_variance * dt;
    P_new[P_index(5, 5)] += _gyro_noise_variance * dt;
    P_new[P_index(6, 6)] += _gyro_noise_variance * dt;
    P_new[P_index(10, 10)] += _accel_noise_variance * dt;
    P_new[P_index(11, 11)] += _accel_noise_variance * dt;
    P_new[P_index(12, 12)] += _accel_noise_variance * dt;
    P_new[P_index(13, 13)] += _bias_noise_variance * dt;
    P_new[P_index(14, 14)] += _bias_noise_variance * dt;
    P_new[P_index(15, 15)] += _bias_noise_variance * dt;
    
    // Copy back
    memcpy(_P, P_new, sizeof(_P));
}

void EKF_AttitudeVelocity::predict_state(float dt)
{
    // Extract current state
    Quaternion q(_state[0], _state[1], _state[2], _state[3]);
    Vector3f omega(_state[4], _state[5], _state[6]);
    Vector3f pos(_state[7], _state[8], _state[9]);
    Vector3f vel(_state[10], _state[11], _state[12]);
    Vector3f bias(_state[13], _state[14], _state[15]);
    
    // Compensate gyro measurement with bias
    Vector3f omega_corrected = omega - bias;
    
    // Update quaternion using quaternion derivative
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
    
    // Update position and velocity
    pos += vel * dt;
    
    // Assume acceleration in body frame is zero (hovering)
    // In reality, this would use accelerometer measurements
    Matrix3f dcm;
    q.rotation_matrix(dcm);
    Vector3f accel_ned = dcm * Vector3f(0, 0, 0) + Vector3f(0, 0, GRAVITY_MSS);
    vel += accel_ned * dt;
    
    // Angular velocity and bias remain constant in prediction
    
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
}

void EKF_AttitudeVelocity::update_imu(const Vector3f& gyro, const Vector3f& accel, float dt)
{
    // Update angular velocity state
    _state[OMEGA_IDX] = gyro.x;
    _state[OMEGA_IDX + 1] = gyro.y;
    _state[OMEGA_IDX + 2] = gyro.z;
    
    // Predict with new measurements
    predict_state(dt);
    
    // Accelerometer update
    // Expected acceleration in body frame
    Quaternion q(_state[0], _state[1], _state[2], _state[3]);
    Matrix3f dcm;
    q.rotation_matrix(dcm);
    
    // Expected measurement: gravity in body frame
    Vector3f h_accel = dcm.transposed() * _gravity_ref;
    
    // Innovation
    Vector3f y_accel = accel - h_accel;
    
    // Measurement Jacobian for accelerometer
    float H_accel[3][STATE_SIZE];
    compute_H_accel_jacobian(H_accel, _state);
    
    // Kalman gain computation
    // K = P * H' * (H * P * H' + R)^-1
    
    // First compute S = H * P * H' + R
    float S[3][3];
    for (uint8_t i = 0; i < 3; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            S[i][j] = 0;
            for (uint8_t k = 0; k < STATE_SIZE; k++) {
                for (uint8_t l = 0; l < STATE_SIZE; l++) {
                    S[i][j] += H_accel[i][k] * P(k, l) * H_accel[j][l];
                }
            }
            if (i == j) {
                S[i][j] += _accel_meas_variance;
            }
        }
    }
    
    // Invert S (3x3 matrix)
    float S_inv[3][3];
    if (!inv3x3(S, S_inv)) {
        return; // Singular matrix, skip update
    }
    
    // Compute Kalman gain K = P * H' * S_inv
    float K[STATE_SIZE][3];
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            K[i][j] = 0;
            for (uint8_t k = 0; k < STATE_SIZE; k++) {
                for (uint8_t l = 0; l < 3; l++) {
                    K[i][j] += P(i, k) * H_accel[l][k] * S_inv[l][j];
                }
            }
        }
    }
    
    // State update: x = x + K * y
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = 0; j < 3; j++) {
            _state[i] += K[i][j] * y_accel[j];
        }
    }
    
    // Covariance update: P = (I - K * H) * P
    float P_new[(STATE_SIZE * (STATE_SIZE + 1)) / 2];
    memcpy(P_new, _P, sizeof(_P));
    
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        for (uint8_t j = i; j < STATE_SIZE; j++) {
            float sum = 0;
            for (uint8_t k = 0; k < 3; k++) {
                sum += K[i][k] * H_accel[k][j];
            }
            P_new[P_index(i, j)] = P(i, j) - sum * P(i, j);
        }
    }
    
    memcpy(_P, P_new, sizeof(_P));
    normalize_quaternion();
}

void EKF_AttitudeVelocity::update_mag(const Vector3f& mag)
{
    // Magnetometer measurement update
    Quaternion q(_state[0], _state[1], _state[2], _state[3]);
    Matrix3f dcm;
    q.rotation_matrix(dcm);
    
    // Expected measurement: Earth's magnetic field in body frame
    Vector3f h_mag = dcm.transposed() * _mag_earth;
    
    // Innovation
    Vector3f y_mag = mag - h_mag;
    
    // Similar Kalman update process as accelerometer
    // (Implementation follows same pattern as update_imu)
}

void EKF_AttitudeVelocity::update_gps(const Vector3f& position, const Vector3f& velocity)
{
    // GPS position update
    Vector3f y_pos = position - Vector3f(_state[POS_IDX], _state[POS_IDX + 1], _state[POS_IDX + 2]);
    
    // Simple position update (H matrix is identity for position states)
    float K_pos = P(POS_IDX, POS_IDX) / (P(POS_IDX, POS_IDX) + _gps_pos_variance);
    
    _state[POS_IDX] += K_pos * y_pos.x;
    _state[POS_IDX + 1] += K_pos * y_pos.y;
    _state[POS_IDX + 2] += K_pos * y_pos.z;
    
    // Update position covariance
    P(POS_IDX, POS_IDX) *= (1 - K_pos);
    P(POS_IDX + 1, POS_IDX + 1) *= (1 - K_pos);
    P(POS_IDX + 2, POS_IDX + 2) *= (1 - K_pos);
    
    // GPS velocity update
    Vector3f y_vel = velocity - Vector3f(_state[VEL_IDX], _state[VEL_IDX + 1], _state[VEL_IDX + 2]);
    
    float K_vel = P(VEL_IDX, VEL_IDX) / (P(VEL_IDX, VEL_IDX) + _gps_vel_variance);
    
    _state[VEL_IDX] += K_vel * y_vel.x;
    _state[VEL_IDX + 1] += K_vel * y_vel.y;
    _state[VEL_IDX + 2] += K_vel * y_vel.z;
    
    // Update velocity covariance
    P(VEL_IDX, VEL_IDX) *= (1 - K_vel);
    P(VEL_IDX + 1, VEL_IDX + 1) *= (1 - K_vel);
    P(VEL_IDX + 2, VEL_IDX + 2) *= (1 - K_vel);
}

Vector3f EKF_AttitudeVelocity::get_euler_angles() const
{
    Quaternion q(_state[0], _state[1], _state[2], _state[3]);
    float roll, pitch, yaw;
    q.to_euler(roll, pitch, yaw);
    return Vector3f(roll, pitch, yaw);
}

void EKF_AttitudeVelocity::normalize_quaternion()
{
    float norm = sqrtf(_state[0]*_state[0] + _state[1]*_state[1] + 
                      _state[2]*_state[2] + _state[3]*_state[3]);
    
    if (norm > 0.0001f) {
        _state[0] /= norm;
        _state[1] /= norm;
        _state[2] /= norm;
        _state[3] /= norm;
    }
}

Quaternion EKF_AttitudeVelocity::quaternion_multiply(const Quaternion& q1, const Quaternion& q2) const
{
    return Quaternion(
        q1.q1*q2.q1 - q1.q2*q2.q2 - q1.q3*q2.q3 - q1.q4*q2.q4,
        q1.q1*q2.q2 + q1.q2*q2.q1 + q1.q3*q2.q4 - q1.q4*q2.q3,
        q1.q1*q2.q3 - q1.q2*q2.q4 + q1.q3*q2.q1 + q1.q4*q2.q2,
        q1.q1*q2.q4 + q1.q2*q2.q3 - q1.q3*q2.q2 + q1.q4*q2.q1
    );
}

void EKF_AttitudeVelocity::compute_F_jacobian(float F[STATE_SIZE][STATE_SIZE], 
                                              const float* x, 
                                              const Vector3f& gyro, 
                                              const Vector3f& accel, 
                                              float dt) const
{
    // Initialize F as identity matrix
    memset(F, 0, sizeof(float) * STATE_SIZE * STATE_SIZE);
    for (uint8_t i = 0; i < STATE_SIZE; i++) {
        F[i][i] = 1.0f;
    }
    
    // Quaternion dynamics Jacobian
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float wx = gyro.x - x[13], wy = gyro.y - x[14], wz = gyro.z - x[15];
    
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
    
    // dq/dbias
    F[0][13] = half_dt * q1;
    F[0][14] = half_dt * q2;
    F[0][15] = half_dt * q3;
    
    F[1][13] = -half_dt * q0;
    F[1][14] = half_dt * q3;
    F[1][15] = -half_dt * q2;
    
    F[2][13] = -half_dt * q3;
    F[2][14] = -half_dt * q0;
    F[2][15] = half_dt * q1;
    
    F[3][13] = half_dt * q2;
    F[3][14] = -half_dt * q1;
    F[3][15] = -half_dt * q0;
    
    // Position dynamics
    F[7][10] = dt;   // dx/dvx
    F[8][11] = dt;   // dy/dvy
    F[9][12] = dt;   // dz/dvz
    
    // Velocity dynamics (simplified - full implementation would include attitude coupling)
    // dv/dq would be non-zero when including accelerometer measurements
}

void EKF_AttitudeVelocity::compute_H_accel_jacobian(float H[3][STATE_SIZE], const float* x) const
{
    // Initialize H to zero
    memset(H, 0, sizeof(float) * 3 * STATE_SIZE);
    
    // Jacobian of gravity vector rotated to body frame with respect to quaternion
    float q0 = x[0], q1 = x[1], q2 = x[2], q3 = x[3];
    float gz = -GRAVITY_MSS;
    
    // Partial derivatives of DCM^T * [0; 0; g] with respect to quaternion
    H[0][0] = 2*gz*(q0*q2 + q1*q3);
    H[0][1] = 2*gz*(q1*q2 - q0*q3);
    H[0][2] = 2*gz*(q0*q0 + q1*q1 - 0.5f);
    H[0][3] = 2*gz*(-q0*q1 + q2*q3);
    
    H[1][0] = 2*gz*(-q0*q1 + q2*q3);
    H[1][1] = 2*gz*(q0*q0 - 0.5f + q2*q2);
    H[1][2] = 2*gz*(q1*q2 + q0*q3);
    H[1][3] = 2*gz*(q0*q2 - q1*q3);
    
    H[2][0] = 2*gz*(q0*q0 - 0.5f + q3*q3);
    H[2][1] = 2*gz*(q1*q3 + q0*q2);
    H[2][2] = 2*gz*(q2*q3 - q0*q1);
    H[2][3] = 2*gz*(q0*q0 - 0.5f + q3*q3);
}

void EKF_AttitudeVelocity::set_process_noise(float gyro_noise, float accel_noise, float bias_noise)
{
    _gyro_noise_variance = gyro_noise * gyro_noise;
    _accel_noise_variance = accel_noise * accel_noise;
    _bias_noise_variance = bias_noise * bias_noise;
}

void EKF_AttitudeVelocity::set_measurement_noise(float gyro_noise, float accel_noise, 
                                                 float mag_noise, float gps_pos_noise, 
                                                 float gps_vel_noise)
{
    _gyro_meas_variance = gyro_noise * gyro_noise;
    _accel_meas_variance = accel_noise * accel_noise;
    _mag_meas_variance = mag_noise * mag_noise;
    _gps_pos_variance = gps_pos_noise * gps_pos_noise;
    _gps_vel_variance = gps_vel_noise * gps_vel_noise;
}

float EKF_AttitudeVelocity::get_attitude_variance() const
{
    return (P(0, 0) + P(1, 1) + P(2, 2) + P(3, 3)) / 4.0f;
}

float EKF_AttitudeVelocity::get_velocity_variance() const
{
    return (P(VEL_IDX, VEL_IDX) + P(VEL_IDX+1, VEL_IDX+1) + P(VEL_IDX+2, VEL_IDX+2)) / 3.0f;
}

float EKF_AttitudeVelocity::get_position_variance() const
{
    return (P(POS_IDX, POS_IDX) + P(POS_IDX+1, POS_IDX+1) + P(POS_IDX+2, POS_IDX+2)) / 3.0f;
}