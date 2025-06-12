#include "unified_estimator.h"
#include <cmath>
#include <cstring>

UnifiedEstimator::UnifiedEstimator() :
    _gravity_ref(0, 0, GRAVITY_MSS),
    _mag_ref(0.2f, 0, 0.4f),
    _aggressive_id_mode(false),
    _last_excitation_ms(0)
{
    // Initialize state
    memset(_x, 0, sizeof(_x));
    _x[IDX_QUAT] = 1.0f;  // Identity quaternion
    
    // Default parameters
    _x[IDX_INERTIA] = 0.02f;     // Ixx
    _x[IDX_INERTIA+1] = 0.02f;   // Iyy  
    _x[IDX_INERTIA+2] = 0.04f;   // Izz
    _x[IDX_THRUST] = 1.5e-5f;     // Thrust coefficient
    _x[IDX_TORQUE] = 3.0e-7f;     // Torque coefficient
    _x[IDX_MASS] = 2.0f;          // Mass (kg)
    _x[IDX_DRAG] = 0.1f;          // Linear drag
    _x[IDX_DRAG+1] = 0.01f;       // Quadratic drag
    
    // Initialize covariance
    memset(_P, 0, sizeof(_P));
    
    // State uncertainties (diagonal)
    for (int i = 0; i < 4; i++) _P[i][i] = 0.01f;      // Quaternion
    for (int i = 4; i < 7; i++) _P[i][i] = 0.1f;       // Angular velocity
    for (int i = 7; i < 10; i++) _P[i][i] = 100.0f;    // Position
    for (int i = 10; i < 13; i++) _P[i][i] = 1.0f;     // Velocity
    for (int i = 13; i < 16; i++) _P[i][i] = 0.01f;    // Gyro bias
    
    // Parameter uncertainties
    _P[IDX_INERTIA][IDX_INERTIA] = 0.001f;     // 10% of nominal
    _P[IDX_INERTIA+1][IDX_INERTIA+1] = 0.001f;
    _P[IDX_INERTIA+2][IDX_INERTIA+2] = 0.001f;
    _P[IDX_THRUST][IDX_THRUST] = (0.3e-5f) * (0.3e-5f);  // 20% uncertainty
    _P[IDX_TORQUE][IDX_TORQUE] = (0.6e-7f) * (0.6e-7f);
    _P[IDX_MASS][IDX_MASS] = 0.04f;  // 200g uncertainty
    _P[IDX_DRAG][IDX_DRAG] = 0.01f;
    _P[IDX_DRAG+1][IDX_DRAG+1] = 0.0001f;
    
    // Process noise - states
    _Q_state[0] = _Q_state[1] = _Q_state[2] = _Q_state[3] = 1e-6f;  // Quaternion
    _Q_state[4] = _Q_state[5] = _Q_state[6] = 0.01f;  // Angular velocity
    _Q_state[7] = _Q_state[8] = _Q_state[9] = 0.0f;   // Position (from velocity)
    _Q_state[10] = _Q_state[11] = _Q_state[12] = 0.1f; // Velocity
    _Q_state[13] = _Q_state[14] = _Q_state[15] = 1e-6f; // Bias drift
    
    // Process noise - parameters (very small - slow variation)
    _Q_param[0] = _Q_param[1] = _Q_param[2] = 1e-8f;  // Inertia
    _Q_param[3] = 1e-12f;  // Thrust coeff
    _Q_param[4] = 1e-14f;  // Torque coeff  
    _Q_param[5] = 1e-6f;   // Mass (can change with fuel/payload)
    _Q_param[6] = _Q_param[7] = 1e-6f;  // Drag
    
    // Measurement noise
    _R_gyro = 0.01f;
    _R_accel = 0.1f;
    _R_mag = 0.1f;
    _R_gps_pos = 5.0f;
    _R_gps_vel = 0.1f;
    _R_baro = 1.0f;
}

void UnifiedEstimator::init(const Vector3f& init_pos, const Quaternion& init_att)
{
    _x[IDX_QUAT] = init_att.q1;
    _x[IDX_QUAT+1] = init_att.q2;
    _x[IDX_QUAT+2] = init_att.q3;
    _x[IDX_QUAT+3] = init_att.q4;
    
    _x[IDX_POS] = init_pos.x;
    _x[IDX_POS+1] = init_pos.y;
    _x[IDX_POS+2] = init_pos.z;
}

void UnifiedEstimator::update(const Vector3f& gyro, const Vector3f& accel, 
                             const float motors[6], float dt)
{
    // Prediction step
    predict(gyro, accel, motors, dt);
    
    // Measurement update with gyro (direct measurement of angular velocity)
    // H matrix is identity for omega states
    for (int i = 0; i < 3; i++) {
        float innovation = gyro[i] - (_x[IDX_OMEGA+i] + _x[IDX_BIAS+i]);
        float S = _P[IDX_OMEGA+i][IDX_OMEGA+i] + _P[IDX_BIAS+i][IDX_BIAS+i] + _R_gyro;
        
        if (S > 1e-6f) {
            // Kalman gain
            float K_omega = _P[IDX_OMEGA+i][IDX_OMEGA+i] / S;
            float K_bias = _P[IDX_BIAS+i][IDX_BIAS+i] / S;
            
            // State update
            _x[IDX_OMEGA+i] += K_omega * innovation;
            _x[IDX_BIAS+i] += K_bias * innovation;
            
            // Covariance update
            _P[IDX_OMEGA+i][IDX_OMEGA+i] *= (1 - K_omega);
            _P[IDX_BIAS+i][IDX_BIAS+i] *= (1 - K_bias);
        }
    }
    
    // Accelerometer update (more complex - involves attitude and thrust model)
    update_accel_measurement(accel, motors);
    
    // Enforce constraints
    enforce_constraints();
    symmetrize_covariance();
    
    // Check if we need excitation for identifiability
    update_process_noise();
}

void UnifiedEstimator::predict(const Vector3f& gyro, const Vector3f& accel, 
                               const float motors[6], float dt)
{
    // Save current state
    float x_prev[AUG_DIM];
    memcpy(x_prev, _x, sizeof(_x));
    
    // Extract states and parameters
    Quaternion q(_x[0], _x[1], _x[2], _x[3]);
    Vector3f omega(_x[4], _x[5], _x[6]);
    Vector3f pos(_x[7], _x[8], _x[9]);
    Vector3f vel(_x[10], _x[11], _x[12]);
    Vector3f bias(_x[13], _x[14], _x[15]);
    
    // Parameters
    float Ixx = _x[IDX_INERTIA];
    float Iyy = _x[IDX_INERTIA+1];
    float Izz = _x[IDX_INERTIA+2];
    float kT = _x[IDX_THRUST];
    float kQ = _x[IDX_TORQUE];
    float mass = _x[IDX_MASS];
    float drag_lin = _x[IDX_DRAG];
    float drag_quad = _x[IDX_DRAG+1];
    
    // Compensate gyro
    Vector3f omega_true = gyro - bias;
    
    // --- State Prediction ---
    
    // Quaternion derivative
    float omega_norm = omega_true.length();
    if (omega_norm > 1e-6f) {
        float half_angle = 0.5f * omega_norm * dt;
        float c = cosf(half_angle);
        float s = sinf(half_angle) / omega_norm;
        
        Quaternion dq(c, s*omega_true.x, s*omega_true.y, s*omega_true.z);
        q = quaternion_multiply(q, dq);
    }
    
    // Angular acceleration from torques
    Vector3f torque = compute_body_torque(motors);
    Vector3f alpha;
    alpha.x = (torque.x - (Izz - Iyy) * omega.y * omega.z) / Ixx;
    alpha.y = (torque.y - (Ixx - Izz) * omega.x * omega.z) / Iyy;
    alpha.z = (torque.z - (Iyy - Ixx) * omega.x * omega.y) / Izz;
    
    omega += alpha * dt;
    
    // Linear dynamics
    Matrix3f R;
    q.rotation_matrix(R);
    
    Vector3f thrust_body = compute_body_thrust(motors);
    Vector3f thrust_world = R * thrust_body;
    
    // Acceleration with drag
    Vector3f drag_force = -(drag_lin * vel + drag_quad * vel.length() * vel);
    Vector3f accel_world = thrust_world / mass + _gravity_ref + drag_force / mass;
    
    pos += vel * dt;
    vel += accel_world * dt;
    
    // --- Parameter Prediction (random walk) ---
    // Parameters don't change in prediction, but we add process noise later
    
    // Store predicted state
    _x[0] = q.q1; _x[1] = q.q2; _x[2] = q.q3; _x[3] = q.q4;
    _x[4] = omega.x; _x[5] = omega.y; _x[6] = omega.z;
    _x[7] = pos.x; _x[8] = pos.y; _x[9] = pos.z;
    _x[10] = vel.x; _x[11] = vel.y; _x[12] = vel.z;
    // Bias and parameters unchanged
    
    quaternion_normalize();
    
    // --- Covariance Prediction ---
    float F[AUG_DIM][AUG_DIM];
    compute_augmented_jacobian(F, gyro, accel, motors, dt);
    
    // P = F * P * F' + Q
    float P_new[AUG_DIM][AUG_DIM];
    
    // F * P * F'
    for (int i = 0; i < AUG_DIM; i++) {
        for (int j = i; j < AUG_DIM; j++) {
            P_new[i][j] = 0;
            for (int k = 0; k < AUG_DIM; k++) {
                for (int l = 0; l < AUG_DIM; l++) {
                    P_new[i][j] += F[i][k] * _P[k][l] * F[j][l];
                }
            }
            P_new[j][i] = P_new[i][j];  // Symmetry
        }
    }
    
    // Add process noise
    for (int i = 0; i < STATE_DIM; i++) {
        P_new[i][i] += _Q_state[i] * dt;
    }
    for (int i = 0; i < PARAM_DIM; i++) {
        P_new[STATE_DIM+i][STATE_DIM+i] += _Q_param[i] * dt;
    }
    
    memcpy(_P, P_new, sizeof(_P));
}

void UnifiedEstimator::compute_augmented_jacobian(float F[AUG_DIM][AUG_DIM],
                                                  const Vector3f& gyro,
                                                  const Vector3f& accel,
                                                  const float motors[6],
                                                  float dt)
{
    // Initialize as identity
    memset(F, 0, sizeof(float) * AUG_DIM * AUG_DIM);
    for (int i = 0; i < AUG_DIM; i++) F[i][i] = 1.0f;
    
    // Extract current values
    float q0 = _x[0], q1 = _x[1], q2 = _x[2], q3 = _x[3];
    float wx = _x[4], wy = _x[5], wz = _x[6];
    float vx = _x[10], vy = _x[11], vz = _x[12];
    
    float Ixx = _x[IDX_INERTIA];
    float Iyy = _x[IDX_INERTIA+1];
    float Izz = _x[IDX_INERTIA+2];
    float mass = _x[IDX_MASS];
    
    // --- State Jacobian ---
    
    // Quaternion dynamics: dq/dq and dq/domega
    F[0][1] = -0.5f * wx * dt;
    F[0][2] = -0.5f * wy * dt;
    F[0][3] = -0.5f * wz * dt;
    F[0][4] = -0.5f * q1 * dt;
    F[0][5] = -0.5f * q2 * dt;
    F[0][6] = -0.5f * q3 * dt;
    
    // (Similar for other quaternion components...)
    
    // Angular velocity dynamics: domega/domega and domega/dI
    // omega_dot = I^-1 * (tau - omega x (I * omega))
    
    // Partial derivatives w.r.t. omega
    F[4][5] = dt * (Izz - Iyy) * wz / Ixx;
    F[4][6] = dt * (Izz - Iyy) * wy / Ixx;
    
    // KEY: Partial derivatives w.r.t. inertia parameters!
    Vector3f torque = compute_body_torque(motors);
    F[4][IDX_INERTIA] = -dt * (torque.x - (Izz - Iyy)*wy*wz) / (Ixx*Ixx);
    F[5][IDX_INERTIA+1] = -dt * (torque.y - (Ixx - Izz)*wx*wz) / (Iyy*Iyy);
    F[6][IDX_INERTIA+2] = -dt * (torque.z - (Iyy - Ixx)*wx*wy) / (Izz*Izz);
    
    // Position dynamics
    F[7][10] = dt;  // dx/dvx
    F[8][11] = dt;  // dy/dvy
    F[9][12] = dt;  // dz/dvz
    
    // Velocity dynamics: dv/dq (attitude affects thrust direction)
    Vector3f thrust = compute_body_thrust(motors);
    float t_mag = thrust.length() / mass;
    
    // Simplified - full derivation is complex
    F[10][0] = dt * 2 * t_mag * (q0*q2 + q1*q3);
    F[11][0] = dt * 2 * t_mag * (-q0*q1 + q2*q3);
    
    // KEY: Velocity depends on mass and thrust coefficient!
    F[10][IDX_MASS] = -dt * thrust.x / (mass*mass);
    F[11][IDX_MASS] = -dt * thrust.y / (mass*mass);
    F[12][IDX_MASS] = -dt * (thrust.z - mass*GRAVITY_MSS) / (mass*mass);
    
    F[10][IDX_THRUST] = dt * compute_thrust_derivative(motors, 0) / mass;
    F[11][IDX_THRUST] = dt * compute_thrust_derivative(motors, 1) / mass;
    F[12][IDX_THRUST] = dt * compute_thrust_derivative(motors, 2) / mass;
    
    // Drag effects on velocity
    F[10][10] = 1 - dt * _x[IDX_DRAG];
    F[11][11] = 1 - dt * _x[IDX_DRAG];
    F[12][12] = 1 - dt * _x[IDX_DRAG];
    
    F[10][IDX_DRAG] = -dt * vx;
    F[11][IDX_DRAG] = -dt * vy;
    F[12][IDX_DRAG] = -dt * vz;
    
    // Parameters don't change (random walk)
    // F[param][param] = 1 (already set)
}

Vector3f UnifiedEstimator::compute_body_thrust(const float motors[6]) const
{
    float total_thrust = 0;
    for (int i = 0; i < 6; i++) {
        total_thrust += _x[IDX_THRUST] * motors[i] * motors[i];
    }
    return Vector3f(0, 0, -total_thrust);  // Down in body frame
}

Vector3f UnifiedEstimator::compute_body_torque(const float motors[6]) const
{
    Vector3f torque(0, 0, 0);
    
    const float arm = 0.25f;  // Arm length
    const float angles[6] = {60, 120, 180, 240, 300, 0};  // Motor angles
    const float dirs[6] = {1, -1, 1, -1, 1, -1};  // Yaw directions
    
    for (int i = 0; i < 6; i++) {
        float thrust = _x[IDX_THRUST] * motors[i] * motors[i];
        float angle_rad = radians(angles[i]);
        
        torque.x -= arm * thrust * sinf(angle_rad);
        torque.y += arm * thrust * cosf(angle_rad);
        torque.z += _x[IDX_TORQUE] * motors[i] * motors[i] * dirs[i];
    }
    
    return torque;
}

void UnifiedEstimator::enforce_constraints()
{
    // Positive inertia
    _x[IDX_INERTIA] = fmaxf(_x[IDX_INERTIA], 0.001f);
    _x[IDX_INERTIA+1] = fmaxf(_x[IDX_INERTIA+1], 0.001f);
    _x[IDX_INERTIA+2] = fmaxf(_x[IDX_INERTIA+2], 0.001f);
    
    // Positive coefficients
    _x[IDX_THRUST] = fmaxf(_x[IDX_THRUST], 1e-6f);
    _x[IDX_TORQUE] = fmaxf(_x[IDX_TORQUE], 1e-8f);
    _x[IDX_MASS] = fmaxf(_x[IDX_MASS], 0.1f);
    _x[IDX_DRAG] = fmaxf(_x[IDX_DRAG], 0.0f);
    _x[IDX_DRAG+1] = fmaxf(_x[IDX_DRAG+1], 0.0f);
}

void UnifiedEstimator::symmetrize_covariance()
{
    for (int i = 0; i < AUG_DIM; i++) {
        for (int j = i+1; j < AUG_DIM; j++) {
            float avg = 0.5f * (_P[i][j] + _P[j][i]);
            _P[i][j] = _P[j][i] = avg;
        }
        // Ensure positive diagonal
        _P[i][i] = fmaxf(_P[i][i], 1e-6f);
    }
}

bool UnifiedEstimator::needs_excitation() const
{
    // Check parameter observability
    float obs_inertia = compute_parameter_observability(IDX_INERTIA);
    float obs_thrust = compute_parameter_observability(IDX_THRUST);
    
    // Need excitation if observability is low
    return (obs_inertia < 0.1f || obs_thrust < 0.1f);
}

float UnifiedEstimator::compute_parameter_observability(uint8_t param_idx) const
{
    // Simple metric: inverse of parameter variance normalized by nominal
    float variance = _P[param_idx][param_idx];
    float nominal = fabsf(_x[param_idx]);
    
    if (nominal < 1e-6f) return 0;
    
    return nominal / sqrtf(variance);
}

void UnifiedEstimator::get_inertia(Vector3f& inertia, Vector3f& std_dev) const
{
    inertia.x = _x[IDX_INERTIA];
    inertia.y = _x[IDX_INERTIA+1];
    inertia.z = _x[IDX_INERTIA+2];
    
    std_dev.x = sqrtf(_P[IDX_INERTIA][IDX_INERTIA]);
    std_dev.y = sqrtf(_P[IDX_INERTIA+1][IDX_INERTIA+1]);
    std_dev.z = sqrtf(_P[IDX_INERTIA+2][IDX_INERTIA+2]);
}

void UnifiedEstimator::update_process_noise()
{
    // Increase parameter process noise if in aggressive ID mode
    if (_aggressive_id_mode) {
        for (int i = 0; i < PARAM_DIM; i++) {
            _Q_param[i] *= 10.0f;
        }
    }
    
    // Reduce noise as parameters converge
    float param_convergence = 0;
    for (int i = STATE_DIM; i < AUG_DIM; i++) {
        param_convergence += _P[i][i];
    }
    param_convergence /= PARAM_DIM;
    
    float scale = fminf(1.0f, param_convergence * 100.0f);
    for (int i = 0; i < PARAM_DIM; i++) {
        _Q_param[i] *= scale;
    }
}

// Additional helper functions...

void UnifiedEstimator::quaternion_normalize()
{
    float norm = sqrtf(_x[0]*_x[0] + _x[1]*_x[1] + _x[2]*_x[2] + _x[3]*_x[3]);
    if (norm > 1e-6f) {
        _x[0] /= norm;
        _x[1] /= norm;
        _x[2] /= norm;
        _x[3] /= norm;
    }
}

Quaternion UnifiedEstimator::quaternion_multiply(const Quaternion& q1, const Quaternion& q2)
{
    return Quaternion(
        q1.q1*q2.q1 - q1.q2*q2.q2 - q1.q3*q2.q3 - q1.q4*q2.q4,
        q1.q1*q2.q2 + q1.q2*q2.q1 + q1.q3*q2.q4 - q1.q4*q2.q3,
        q1.q1*q2.q3 - q1.q2*q2.q4 + q1.q3*q2.q1 + q1.q4*q2.q2,
        q1.q1*q2.q4 + q1.q2*q2.q3 - q1.q3*q2.q2 + q1.q4*q2.q1
    );
}

// Simplified implementations for other functions...
void UnifiedEstimator::update_accel_measurement(const Vector3f& accel, const float motors[6])
{
    // Implementation would compute expected acceleration from current state
    // and motor commands, then update state based on innovation
}

float UnifiedEstimator::compute_thrust_derivative(const float motors[6], int axis) const
{
    // Derivative of thrust w.r.t thrust coefficient
    float sum = 0;
    for (int i = 0; i < 6; i++) {
        sum += motors[i] * motors[i];
    }
    return (axis == 2) ? -sum : 0;  // Only Z-axis thrust
}

// Additional method implementations

Quaternion UnifiedEstimator::get_attitude() const
{
    return Quaternion(_x[0], _x[1], _x[2], _x[3]);
}

void UnifiedEstimator::get_thrust_coeff(float& coeff, float& std_dev) const
{
    coeff = _x[IDX_THRUST];
    std_dev = sqrtf(_P[IDX_THRUST][IDX_THRUST]);
}

void UnifiedEstimator::get_mass(float& mass, float& std_dev) const
{
    mass = _x[IDX_MASS];
    std_dev = sqrtf(_P[IDX_MASS][IDX_MASS]);
}

void UnifiedEstimator::update_magnetometer(const Vector3f& mag)
{
    // TODO: Implement magnetometer update
    // Would compute expected magnetic field from attitude
    // and update state accordingly
}

void UnifiedEstimator::update_gps(const Vector3f& pos, const Vector3f& vel)
{
    // Simple position/velocity update
    for (int i = 0; i < 3; i++) {
        // Position update
        float pos_innovation = pos[i] - _x[IDX_POS + i];
        float pos_variance = _P[IDX_POS + i][IDX_POS + i];
        float K_pos = pos_variance / (pos_variance + _R_gps_pos);
        _x[IDX_POS + i] += K_pos * pos_innovation;
        _P[IDX_POS + i][IDX_POS + i] *= (1 - K_pos);
        
        // Velocity update
        float vel_innovation = vel[i] - _x[IDX_VEL + i];
        float vel_variance = _P[IDX_VEL + i][IDX_VEL + i];
        float K_vel = vel_variance / (vel_variance + _R_gps_vel);
        _x[IDX_VEL + i] += K_vel * vel_innovation;
        _P[IDX_VEL + i][IDX_VEL + i] *= (1 - K_vel);
    }
}

void UnifiedEstimator::update_baro(float altitude)
{
    // Update Z position from barometer
    float innovation = -altitude - _x[IDX_POS + 2];  // NED frame
    float variance = _P[IDX_POS + 2][IDX_POS + 2];
    float K = variance / (variance + _R_baro);
    _x[IDX_POS + 2] += K * innovation;
    _P[IDX_POS + 2][IDX_POS + 2] *= (1 - K);
}

Vector3f UnifiedEstimator::get_excitation_command() const
{
    // Generate excitation signal for better parameter observability
    float time = AP_HAL::millis() * 0.001f;
    float amplitude = 0.1f;  // 10% excitation
    
    // Multi-sine excitation at different frequencies
    Vector3f excitation;
    excitation.x = amplitude * sinf(2 * M_PI * 1.0f * time);
    excitation.y = amplitude * sinf(2 * M_PI * 1.5f * time);
    excitation.z = amplitude * sinf(2 * M_PI * 0.5f * time);
    
    return excitation;
}

void UnifiedEstimator::set_parameter_drift_rate(float drift_rate)
{
    // Set process noise for parameters
    for (int i = 0; i < PARAM_DIM; i++) {
        _Q_param[i] = drift_rate;
    }
}

void UnifiedEstimator::set_initial_parameter_uncertainty(float uncertainty)
{
    // Set initial covariance for parameters
    for (int i = STATE_DIM; i < AUG_DIM; i++) {
        _P[i][i] = uncertainty * uncertainty;
    }
}

// ExcitationController implementation

Vector3f ExcitationController::generate_excitation(float amplitude, float duration)
{
    // Generate optimal excitation for parameter identification
    uint32_t current_ms = AP_HAL::millis();
    
    if (_excitation_start_ms == 0) {
        _excitation_start_ms = current_ms;
    }
    
    float elapsed = (current_ms - _excitation_start_ms) * 0.001f;
    
    if (elapsed > duration) {
        _excitation_start_ms = 0;
        return Vector3f(0, 0, 0);
    }
    
    return optimize_for_identifiability() * amplitude;
}

bool ExcitationController::is_sufficiently_excited() const
{
    // Check if system has enough excitation for identification
    return _estimator->needs_excitation() == false;
}

Vector3f ExcitationController::optimize_for_identifiability()
{
    // Use gradient of Fisher information to find best excitation
    // This is a simplified version - full implementation would compute
    // gradients of observability gramian
    
    Vector3f excitation;
    float time = AP_HAL::millis() * 0.001f;
    
    // Multi-frequency excitation
    excitation.x = sinf(2 * M_PI * 2.0f * time);
    excitation.y = sinf(2 * M_PI * 3.0f * time + M_PI/4);
    excitation.z = sinf(2 * M_PI * 1.0f * time + M_PI/2);
    
    return excitation;
}