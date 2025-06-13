#include "velocity_feedback_controller.h"

// Stub HAL for standalone compilation
#ifndef ARDUPILOT_BUILD
namespace {
    struct HALStub {
        struct Console {
            void printf(const char* fmt, ...) {}
        } console_stub;
        Console* console = &console_stub;
    } hal_stub;
}
#define hal hal_stub
#define AP_HAL_millis() 0
#define AP_HAL_micros64() 0
#else
#include <AP_HAL/AP_HAL.h>
#include <AP_Logger/AP_Logger.h>
extern const AP_HAL::HAL& hal;
#define AP_HAL_millis() AP_HAL::millis()
#define AP_HAL_micros64() AP_HAL::micros64()
#endif

VelocityFeedbackController::VelocityFeedbackController() :
    _ekf(nullptr),
    _sysid(nullptr),
    _mode(MODE_VELOCITY),
    _velocity_target(0, 0, 0),
    _position_target(0, 0, 0),
    _attitude_target(1, 0, 0, 0),
    _angular_rate_target(0, 0, 0),
    _velocity_integral(0, 0, 0),
    _position_integral(0, 0, 0),
    _attitude_integral(0, 0, 0),
    _rate_integral(0, 0, 0),
    _velocity_error_prev(0, 0, 0),
    _position_error_prev(0, 0, 0),
    _attitude_error_prev(0, 0, 0),
    _rate_error_prev(0, 0, 0),
    _disturbance_estimate(0, 0, 0),
    _disturbance_integral(0, 0, 0)
{
    // Initialize gains with default values
    _gains.vel_p = Vector3f(5.0f, 5.0f, 10.0f);
    _gains.vel_i = Vector3f(1.0f, 1.0f, 3.0f);
    _gains.vel_d = Vector3f(0.5f, 0.5f, 0.8f);
    _gains.vel_ff = Vector3f(0.1f, 0.1f, 0.2f);
    
    _gains.pos_p = Vector3f(1.0f, 1.0f, 2.0f);
    _gains.pos_i = Vector3f(0.1f, 0.1f, 0.2f);
    _gains.pos_d = Vector3f(0.2f, 0.2f, 0.3f);
    
    _gains.att_p = Vector3f(6.0f, 6.0f, 4.0f);
    _gains.att_i = Vector3f(0.5f, 0.5f, 0.3f);
    _gains.att_d = Vector3f(0.1f, 0.1f, 0.05f);
    
    _gains.rate_p = Vector3f(0.15f, 0.15f, 0.1f);
    _gains.rate_i = Vector3f(0.05f, 0.05f, 0.02f);
    _gains.rate_d = Vector3f(0.003f, 0.003f, 0.001f);
    _gains.rate_ff = Vector3f(0.02f, 0.02f, 0.01f);
    
    _gains.max_tilt_angle = radians(45.0f);
    _gains.max_velocity_xy = 10.0f;  // m/s
    _gains.max_velocity_z = 5.0f;    // m/s
    _gains.max_angular_rate = radians(200.0f);  // rad/s
    
    // Anti-windup limits
    _velocity_integral_limit = Vector3f(2.0f, 2.0f, 4.0f);
    _attitude_integral_limit = Vector3f(0.5f, 0.5f, 0.3f);
    
    // Reference model parameters
    _ref_model.natural_freq = 2.0f;  // rad/s
    _ref_model.damping_ratio = 0.7f;
    
    // Initialize motor outputs
    for (uint8_t i = 0; i < 6; i++) {
        _motor_outputs[i] = 0;
    }
    
    // Performance metrics
    _performance.velocity_error_mag = 0;
    _performance.position_error_mag = 0;
    _performance.attitude_error_mag = 0;
    _performance.control_effort = 0;
    _performance.update_count = 0;
}

void VelocityFeedbackController::init(EKF_AttitudeVelocity* ekf, HexacopterSysID* sysid)
{
    _ekf = ekf;
    _sysid = sysid;
    
    // Reset integrators
    _velocity_integral.zero();
    _position_integral.zero();
    _attitude_integral.zero();
    _rate_integral.zero();
    
    // Initialize reference model
    if (_ekf != nullptr) {
        _ref_model.velocity = _ekf->get_velocity();
        _ref_model.position = _ekf->get_position();
    }
}

void VelocityFeedbackController::update(float dt)
{
    if (_ekf == nullptr || _sysid == nullptr) {
        return;
    }
    
    // Get current state from EKF
    Vector3f position_current = _ekf->get_position();
    Vector3f velocity_current = _ekf->get_velocity();
    Quaternion attitude_current = _ekf->get_quaternion();
    Vector3f rate_current = _ekf->get_angular_velocity();
    
    // Update reference model
    update_reference_model(dt);
    
    // Update adaptive gains if system ID is running
    if (_sysid->is_running()) {
        update_adaptive_gains();
    }
    
    // Outer control loops
    switch (_mode) {
        case MODE_POSITION:
            position_control(position_current, dt);
            velocity_control(velocity_current, dt);
            break;
            
        case MODE_VELOCITY:
            velocity_control(velocity_current, dt);
            break;
            
        case MODE_STABILIZE:
            // Direct attitude control
            _attitude_target = Quaternion(1, 0, 0, 0);  // Level
            break;
            
        case MODE_ACRO:
            // Direct rate control
            break;
    }
    
    // Inner control loops (always active except in ACRO mode)
    if (_mode != MODE_ACRO) {
        attitude_control(attitude_current, dt);
    }
    rate_control(rate_current, dt);
    
    // Update performance metrics
    _performance.update_count++;
    
    // Log controller state
    if (_performance.update_count % 10 == 0) {
        log_controller_state();
    }
}

void VelocityFeedbackController::position_control(const Vector3f& position_current, float dt)
{
    // Position error
    Vector3f position_error = _position_target - position_current;
    
    // P term
    Vector3f vel_p = position_error * _gains.pos_p;
    
    // I term with anti-windup
    _position_integral += position_error * dt;
    apply_anti_windup(_position_integral, position_error, Vector3f(5.0f, 5.0f, 5.0f));
    Vector3f vel_i = _position_integral * _gains.pos_i;
    
    // D term
    Vector3f position_error_rate = (position_error - _position_error_prev) / dt;
    Vector3f vel_d = position_error_rate * _gains.pos_d;
    _position_error_prev = position_error;
    
    // Generate velocity setpoint
    _velocity_target = vel_p + vel_i + vel_d;
    
    // Apply velocity limits
    float vel_xy = norm(_velocity_target.x, _velocity_target.y);
    if (vel_xy > _gains.max_velocity_xy) {
        float scale = _gains.max_velocity_xy / vel_xy;
        _velocity_target.x *= scale;
        _velocity_target.y *= scale;
    }
    _velocity_target.z = constrain_float(_velocity_target.z, -_gains.max_velocity_z, _gains.max_velocity_z);
    
    // Update performance
    _performance.position_error_mag = position_error.length();
}

void VelocityFeedbackController::velocity_control(const Vector3f& velocity_current, float dt)
{
    // Velocity error
    Vector3f velocity_error = _velocity_target - velocity_current;
    
    // P term
    Vector3f acc_p = velocity_error * _gains.vel_p;
    
    // I term with anti-windup
    _velocity_integral += velocity_error * dt;
    apply_anti_windup(_velocity_integral, velocity_error, _velocity_integral_limit);
    Vector3f acc_i = _velocity_integral * _gains.vel_i;
    
    // D term
    Vector3f velocity_error_rate = (velocity_error - _velocity_error_prev) / dt;
    Vector3f acc_d = velocity_error_rate * _gains.vel_d;
    _velocity_error_prev = velocity_error;
    
    // Feed-forward term
    Vector3f acc_ff = compute_feedforward_acceleration(_velocity_target);
    
    // Total acceleration command
    Vector3f acc_cmd = acc_p + acc_i + acc_d + acc_ff;
    
    // Apply disturbance compensation
    acc_cmd -= _disturbance_estimate;
    
    // Convert acceleration command to attitude setpoint
    _attitude_target = velocity_to_attitude_setpoint(acc_cmd, acc_ff);
    
    // Apply tilt limit
    limit_tilt(_attitude_target);
    
    // Update performance
    _performance.velocity_error_mag = velocity_error.length();
}

void VelocityFeedbackController::attitude_control(const Quaternion& attitude_current, float dt)
{
    // Attitude error (quaternion)
    Vector3f attitude_error = quaternion_error(_attitude_target, attitude_current);
    
    // P term
    Vector3f rate_p = attitude_error * _gains.att_p;
    
    // I term with anti-windup
    _attitude_integral += attitude_error * dt;
    apply_anti_windup(_attitude_integral, attitude_error, _attitude_integral_limit);
    Vector3f rate_i = _attitude_integral * _gains.att_i;
    
    // D term
    Vector3f attitude_error_rate = (attitude_error - _attitude_error_prev) / dt;
    Vector3f rate_d = attitude_error_rate * _gains.att_d;
    _attitude_error_prev = attitude_error;
    
    // Feed-forward term
    Vector3f rate_ff = compute_feedforward_rates(_attitude_target);
    
    // Generate angular rate setpoint
    _angular_rate_target = rate_p + rate_i + rate_d + rate_ff;
    
    // Apply rate limits
    _angular_rate_target.x = constrain_float(_angular_rate_target.x, -_gains.max_angular_rate, _gains.max_angular_rate);
    _angular_rate_target.y = constrain_float(_angular_rate_target.y, -_gains.max_angular_rate, _gains.max_angular_rate);
    _angular_rate_target.z = constrain_float(_angular_rate_target.z, -_gains.max_angular_rate, _gains.max_angular_rate);
    
    // Update performance
    _performance.attitude_error_mag = attitude_error.length();
}

void VelocityFeedbackController::rate_control(const Vector3f& rate_current, float dt)
{
    // Rate error
    Vector3f rate_error = _angular_rate_target - rate_current;
    
    // Account for gyro bias from EKF
    Vector3f gyro_bias = _ekf->get_gyro_bias();
    rate_error += gyro_bias;
    
    // P term
    Vector3f torque_p = rate_error * _gains.rate_p;
    
    // I term
    _rate_integral += rate_error * dt;
    Vector3f torque_i = _rate_integral * _gains.rate_i;
    
    // D term
    Vector3f rate_error_rate = (rate_error - _rate_error_prev) / dt;
    Vector3f torque_d = rate_error_rate * _gains.rate_d;
    _rate_error_prev = rate_error;
    
    // Feed-forward using identified system parameters
    Matrix3f inertia = _sysid->get_inertia_matrix();
    Vector3f torque_ff = inertia * _angular_rate_target * _gains.rate_ff;
    
    // Total torque command
    Vector3f torque_cmd = torque_p + torque_i + torque_d + torque_ff;
    
    // Thrust command using identified mass
    float mass = _sysid ? _sysid->get_vehicle_mass() : 2.0f;  // Use identified mass or default
    float hover_thrust = mass * GRAVITY_MSS;
    float thrust_cmd = hover_thrust;  // Base thrust
    
    // Add vertical control
    if (_mode == MODE_VELOCITY || _mode == MODE_POSITION) {
        float vertical_acc = _velocity_integral.z * _gains.vel_i.z;
        thrust_cmd += mass * vertical_acc;
    }
    
    Vector3f thrust_body(0, 0, -thrust_cmd);
    
    // Mix motors
    mix_motors(thrust_body, torque_cmd);
    
    // Update performance
    _performance.control_effort = 0;
    for (uint8_t i = 0; i < 6; i++) {
        _performance.control_effort += _motor_outputs[i];
    }
    _performance.control_effort /= 6.0f;
}

Quaternion VelocityFeedbackController::velocity_to_attitude_setpoint(const Vector3f& acc_cmd, const Vector3f& acc_ff)
{
    // Convert acceleration command to desired attitude
    // Assumes acceleration is achieved by tilting the vehicle
    
    // Total acceleration including gravity
    Vector3f acc_total = acc_cmd;
    acc_total.z -= GRAVITY_MSS;
    
    // Normalize to get desired thrust direction
    float thrust_mag = acc_total.length();
    if (thrust_mag < 0.1f) {
        // Near zero acceleration, maintain level
        return Quaternion(1, 0, 0, 0);
    }
    
    Vector3f thrust_dir = acc_total / thrust_mag;
    
    // Convert thrust direction to attitude
    // Desired body Z axis (thrust) should align with thrust_dir
    Vector3f z_body = -thrust_dir;
    
    // Choose yaw to maintain current heading
    Vector3f euler_current = _ekf->get_euler_angles();
    float yaw = euler_current.z;
    
    // Desired body X axis (forward)
    Vector3f x_body(cosf(yaw), sinf(yaw), 0);
    x_body = x_body - z_body * (x_body * z_body);
    x_body.normalize();
    
    // Body Y axis from cross product
    Vector3f y_body = z_body % x_body;
    
    // Create rotation matrix
    Matrix3f dcm;
    dcm.a = x_body;
    dcm.b = y_body;
    dcm.c = z_body;
    
    // Convert to quaternion
    Quaternion q;
    q.from_rotation_matrix(dcm);
    
    return q;
}

void VelocityFeedbackController::mix_motors(const Vector3f& thrust_body, const Vector3f& torque_body)
{
    // Get system parameters
    float thrust_coeff = _sysid ? _sysid->get_thrust_coefficient() : 1.5e-5f;
    float torque_coeff = _sysid ? _sysid->get_torque_coefficient() : 3.0e-7f;
    
    // Motor mixing matrix for hexacopter X configuration
    // Each motor contributes to thrust and torques
    const float arm_length = 0.25f;  // meters
    const float motor_angles[6] = {
        radians(60.0f),   // Motor 1
        radians(120.0f),  // Motor 2
        radians(180.0f),  // Motor 3
        radians(240.0f),  // Motor 4
        radians(300.0f),  // Motor 5
        radians(0.0f)     // Motor 6
    };
    
    // Yaw moment directions (alternating CW/CCW)
    const float yaw_direction[6] = {1, -1, 1, -1, 1, -1};
    
    // Build mixing matrix
    // [Thrust; Roll; Pitch; Yaw] = M * [motor1; motor2; ...; motor6]
    float M[4][6];
    
    for (uint8_t i = 0; i < 6; i++) {
        // Thrust contribution (all motors contribute equally)
        M[0][i] = thrust_coeff;
        
        // Roll moment contribution
        M[1][i] = -arm_length * thrust_coeff * sinf(motor_angles[i]);
        
        // Pitch moment contribution
        M[2][i] = arm_length * thrust_coeff * cosf(motor_angles[i]);
        
        // Yaw moment contribution
        M[3][i] = torque_coeff * yaw_direction[i];
    }
    
    // Desired outputs
    float desired[4] = {
        -thrust_body.z,  // Total thrust (body Z is down)
        torque_body.x,   // Roll torque
        torque_body.y,   // Pitch torque
        torque_body.z    // Yaw torque
    };
    
    // Solve for motor commands using pseudo-inverse
    // For hexacopter, we have 6 motors and 4 constraints, so system is over-determined
    // Use least squares solution
    
    // Simple allocation (can be improved with proper pseudo-inverse)
    float baseline = desired[0] / 6.0f;  // Equal thrust distribution
    
    for (uint8_t i = 0; i < 6; i++) {
        _motor_outputs[i] = baseline;
        
        // Add differential thrust for torques
        _motor_outputs[i] += desired[1] * M[1][i] / (thrust_coeff * arm_length * 3.0f);
        _motor_outputs[i] += desired[2] * M[2][i] / (thrust_coeff * arm_length * 3.0f);
        _motor_outputs[i] += desired[3] * M[3][i] / (torque_coeff * 6.0f);
        
        // Normalize to 0-1 range
        _motor_outputs[i] = constrain_float(_motor_outputs[i] / (thrust_coeff * 1000.0f), 0.0f, 1.0f);
    }
}

void VelocityFeedbackController::limit_tilt(Quaternion& q_sp)
{
    // Extract tilt angle from quaternion
    Matrix3f dcm;
    q_sp.rotation_matrix(dcm);
    
    // Body Z axis in NED frame
    Vector3f z_body = dcm.c;
    
    // Tilt angle from vertical
    float tilt_angle = acosf(-z_body.z);
    
    if (tilt_angle > _gains.max_tilt_angle) {
        // Scale tilt to maximum
        float scale = _gains.max_tilt_angle / tilt_angle;
        
        // Interpolate quaternion toward level
        Quaternion q_level(1, 0, 0, 0);
        float alpha = 1.0f - scale;
        
        q_sp = q_sp * (1.0f - alpha) + q_level * alpha;
        q_sp.normalize();
    }
}

void VelocityFeedbackController::apply_anti_windup(Vector3f& integral, const Vector3f& error, const Vector3f& limit)
{
    // Simple anti-windup: stop integrating if we're at the limit and error has same sign
    for (uint8_t i = 0; i < 3; i++) {
        if (fabsf(integral[i]) > limit[i]) {
            if ((integral[i] > 0 && error[i] > 0) || (integral[i] < 0 && error[i] < 0)) {
                integral[i] = constrain_float(integral[i], -limit[i], limit[i]);
            }
        }
    }
}

Vector3f VelocityFeedbackController::quaternion_error(const Quaternion& q_sp, const Quaternion& q)
{
    // Quaternion error: q_error = q_sp * q^-1
    Quaternion q_inv = q.inverse();
    Quaternion q_error = q_sp * q_inv;
    
    // Ensure scalar part is positive
    if (q_error.q1 < 0) {
        q_error = -q_error;
    }
    
    // Convert to axis-angle representation
    float angle = 2.0f * acosf(constrain_float(q_error.q1, -1.0f, 1.0f));
    Vector3f axis(q_error.q2, q_error.q3, q_error.q4);
    
    if (angle > 0.001f) {
        axis /= sinf(angle / 2.0f);
        return axis * angle;
    } else {
        return Vector3f(0, 0, 0);
    }
}

void VelocityFeedbackController::update_adaptive_gains()
{
    // Use system identification results to update controller gains
    Matrix3f inertia = _sysid->get_inertia_matrix();
    Vector3f damping = _sysid->get_damping_coefficients();
    float thrust_coeff = _sysid->get_thrust_coefficient();
    float torque_coeff = _sysid->get_torque_coefficient();
    
    // Update rate controller gains based on identified inertia
    // Using model-based tuning with desired bandwidth
    float bandwidth_roll = 10.0f;  // rad/s
    float bandwidth_pitch = 10.0f;
    float bandwidth_yaw = 5.0f;
    
    // P gains: Kp = I * bandwidth^2
    _gains.rate_p.x = inertia.a.x * bandwidth_roll * bandwidth_roll;
    _gains.rate_p.y = inertia.b.y * bandwidth_pitch * bandwidth_pitch;
    _gains.rate_p.z = inertia.c.z * bandwidth_yaw * bandwidth_yaw;
    
    // D gains: Kd = 2 * I * zeta * bandwidth - damping
    float zeta = 0.7f;  // Damping ratio
    _gains.rate_d.x = 2.0f * inertia.a.x * zeta * bandwidth_roll - damping.x;
    _gains.rate_d.y = 2.0f * inertia.b.y * zeta * bandwidth_pitch - damping.y;
    _gains.rate_d.z = 2.0f * inertia.c.z * zeta * bandwidth_yaw - damping.z;
    
    // Ensure positive D gains
    _gains.rate_d.x = fmaxf(_gains.rate_d.x, 0.001f);
    _gains.rate_d.y = fmaxf(_gains.rate_d.y, 0.001f);
    _gains.rate_d.z = fmaxf(_gains.rate_d.z, 0.001f);
    
    // I gains: Ki = bandwidth^3 / 10 (conservative)
    _gains.rate_i.x = bandwidth_roll * bandwidth_roll * bandwidth_roll / 10.0f;
    _gains.rate_i.y = bandwidth_pitch * bandwidth_pitch * bandwidth_pitch / 10.0f;
    _gains.rate_i.z = bandwidth_yaw * bandwidth_yaw * bandwidth_yaw / 10.0f;
    
    // Update velocity controller gains based on thrust characteristics
    // Maximum acceleration = total_thrust / mass
    float mass = _sysid->get_vehicle_mass();
    float max_thrust = 6.0f * thrust_coeff * 1000.0f * 1000.0f;  // 6 motors at max
    float max_accel = max_thrust / mass;
    
    // Velocity P gain scales with available acceleration
    float vel_bandwidth = 2.0f;  // Desired velocity loop bandwidth
    _gains.vel_p = _gains.vel_p * (max_accel / (GRAVITY_MSS * 2.0f));  // Normalize to 2g
    
    // Feed-forward gains based on identified thrust
    _gains.vel_ff = _gains.vel_ff * (thrust_coeff / 1.5e-5f);  // Normalize to nominal
}

void VelocityFeedbackController::update_disturbance_observer(const Vector3f& acceleration_measured, 
                                                             const Vector3f& acceleration_expected, 
                                                             float dt)
{
    // Estimate external disturbances
    Vector3f disturbance = acceleration_measured - acceleration_expected;
    
    // Low-pass filter the disturbance estimate
    const float tau = 1.0f;  // Time constant
    float alpha = dt / (tau + dt);
    
    _disturbance_estimate = _disturbance_estimate * (1 - alpha) + disturbance * alpha;
    
    // Integrate disturbance for bias estimation
    _disturbance_integral += disturbance * dt;
    _disturbance_integral *= 0.999f;  // Slow decay
}

void VelocityFeedbackController::update_reference_model(float dt)
{
    // Second-order reference model for smooth trajectory generation
    float omega_n = _ref_model.natural_freq;
    float zeta = _ref_model.damping_ratio;
    
    // Reference model dynamics
    Vector3f vel_error = _velocity_target - _ref_model.velocity;
    Vector3f vel_dot = omega_n * omega_n * vel_error - 2 * zeta * omega_n * _ref_model.velocity;
    
    _ref_model.velocity += vel_dot * dt;
    _ref_model.position += _ref_model.velocity * dt;
}

Vector3f VelocityFeedbackController::compute_feedforward_acceleration(const Vector3f& velocity_target)
{
    // Compute expected acceleration for trajectory following
    // This could include drag compensation, trajectory derivatives, etc.
    
    // Simple drag model
    const float drag_coeff = 0.1f;
    Vector3f drag_acc = -velocity_target * drag_coeff;
    
    return drag_acc;
}

Vector3f VelocityFeedbackController::compute_feedforward_rates(const Quaternion& attitude_target)
{
    // Compute expected angular rates for attitude trajectory
    // This would typically use the derivative of the attitude setpoint
    
    // For now, return zero (assumes step changes in attitude)
    return Vector3f(0, 0, 0);
}

void VelocityFeedbackController::set_velocity_target(const Vector3f& velocity_ned)
{
    _velocity_target = velocity_ned;
    
    // Apply limits
    float vel_xy = norm(_velocity_target.x, _velocity_target.y);
    if (vel_xy > _gains.max_velocity_xy) {
        float scale = _gains.max_velocity_xy / vel_xy;
        _velocity_target.x *= scale;
        _velocity_target.y *= scale;
    }
    _velocity_target.z = constrain_float(_velocity_target.z, -_gains.max_velocity_z, _gains.max_velocity_z);
}

void VelocityFeedbackController::set_position_target(const Vector3f& position_ned)
{
    _position_target = position_ned;
}

void VelocityFeedbackController::get_motor_outputs(float motors[6]) const
{
    memcpy(motors, _motor_outputs, sizeof(_motor_outputs));
}

void VelocityFeedbackController::log_controller_state()
{
#if HAL_LOGGING_ENABLED && defined(ARDUPILOT_BUILD)
    AP::logger().WriteStreaming("VCTL",
        "TimeUS,Mode,VelErrX,VelErrY,VelErrZ,AttErrR,AttErrP,AttErrY,M1,M2,M3,M4,M5,M6",
        "QBffffffffffff",
        AP_HAL_micros64(),
        (uint8_t)_mode,
        _velocity_error_prev.x,
        _velocity_error_prev.y,
        _velocity_error_prev.z,
        _attitude_error_prev.x,
        _attitude_error_prev.y,
        _attitude_error_prev.z,
        _motor_outputs[0],
        _motor_outputs[1],
        _motor_outputs[2],
        _motor_outputs[3],
        _motor_outputs[4],
        _motor_outputs[5]
    );
#endif
}