#ifndef VELOCITY_FEEDBACK_CONTROLLER_H
#define VELOCITY_FEEDBACK_CONTROLLER_H

#ifdef ARDUPILOT_BUILD
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#else
#include "ap_math_compat.h"
#endif
#include "ekf_attitude_velocity.h"
#include "hexacopter_sysid.h"

/*
 * Velocity Feedback Controller for Hexacopter
 * 
 * This controller implements a cascaded control architecture:
 * 1. Outer loop: Velocity control (generates attitude setpoints)
 * 2. Inner loop: Attitude control (generates motor commands)
 * 
 * Features:
 * - Adaptive control using system identification results
 * - Feed-forward compensation
 * - Anti-windup for integral terms
 * - Reference model following
 * - Disturbance observer
 */

class VelocityFeedbackController {
public:
    VelocityFeedbackController();

    // Initialize controller
    void init(EKF_AttitudeVelocity* ekf, HexacopterSysID* sysid);

    // Main control update
    void update(float dt);

    // Set velocity command (NED frame)
    void set_velocity_target(const Vector3f& velocity_ned);
    
    // Set position hold mode
    void set_position_target(const Vector3f& position_ned);
    
    // Get motor outputs (normalized 0-1)
    void get_motor_outputs(float motors[6]) const;

    // Controller modes
    enum ControlMode {
        MODE_VELOCITY,
        MODE_POSITION,
        MODE_ACRO,
        MODE_STABILIZE
    };
    
    void set_mode(ControlMode mode) { _mode = mode; }
    ControlMode get_mode() const { return _mode; }

    // Tuning parameters
    struct Gains {
        // Velocity controller gains
        Vector3f vel_p;     // Proportional
        Vector3f vel_i;     // Integral
        Vector3f vel_d;     // Derivative
        Vector3f vel_ff;    // Feed-forward
        
        // Position controller gains
        Vector3f pos_p;
        Vector3f pos_i;
        Vector3f pos_d;
        
        // Attitude controller gains
        Vector3f att_p;
        Vector3f att_i;
        Vector3f att_d;
        
        // Angular rate controller gains
        Vector3f rate_p;
        Vector3f rate_i;
        Vector3f rate_d;
        Vector3f rate_ff;
        
        // Limits
        float max_tilt_angle;       // Maximum tilt in radians
        float max_velocity_xy;      // Maximum horizontal velocity
        float max_velocity_z;       // Maximum vertical velocity
        float max_angular_rate;     // Maximum angular rate
    };
    
    void set_gains(const Gains& gains) { _gains = gains; }
    const Gains& get_gains() const { return _gains; }

    // Performance metrics
    struct Performance {
        float velocity_error_mag;
        float position_error_mag;
        float attitude_error_mag;
        float control_effort;
        uint32_t update_count;
    };
    
    const Performance& get_performance() const { return _performance; }

private:
    // References
    EKF_AttitudeVelocity* _ekf;
    HexacopterSysID* _sysid;
    
    // Control mode
    ControlMode _mode;
    
    // Targets
    Vector3f _velocity_target;
    Vector3f _position_target;
    Quaternion _attitude_target;
    Vector3f _angular_rate_target;
    
    // Control gains
    Gains _gains;
    
    // Integral terms
    Vector3f _velocity_integral;
    Vector3f _position_integral;
    Vector3f _attitude_integral;
    Vector3f _rate_integral;
    
    // Previous errors for derivative
    Vector3f _velocity_error_prev;
    Vector3f _position_error_prev;
    Vector3f _attitude_error_prev;
    Vector3f _rate_error_prev;
    
    // Anti-windup
    Vector3f _velocity_integral_limit;
    Vector3f _attitude_integral_limit;
    
    // Motor outputs
    float _motor_outputs[6];
    
    // Performance tracking
    Performance _performance;
    
    // Disturbance observer
    Vector3f _disturbance_estimate;
    Vector3f _disturbance_integral;
    
    // Reference model
    struct ReferenceModel {
        Vector3f velocity;
        Vector3f position;
        float natural_freq;
        float damping_ratio;
    } _ref_model;
    
    // Control methods
    void velocity_control(const Vector3f& velocity_current, float dt);
    void position_control(const Vector3f& position_current, float dt);
    void attitude_control(const Quaternion& attitude_current, float dt);
    void rate_control(const Vector3f& rate_current, float dt);
    
    // Generate attitude setpoint from velocity error
    Quaternion velocity_to_attitude_setpoint(const Vector3f& velocity_error, const Vector3f& velocity_ff);
    
    // Motor mixing
    void mix_motors(const Vector3f& thrust_body, const Vector3f& torque_body);
    
    // Utility functions
    void limit_tilt(Quaternion& q_sp);
    void apply_anti_windup(Vector3f& integral, const Vector3f& error, const Vector3f& limit);
    Vector3f quaternion_error(const Quaternion& q_sp, const Quaternion& q);
    
    // Adaptive control
    void update_adaptive_gains();
    
    // Disturbance observer
    void update_disturbance_observer(const Vector3f& acceleration_measured, 
                                    const Vector3f& acceleration_expected, 
                                    float dt);
    
    // Reference model update
    void update_reference_model(float dt);
    
    // Feed-forward computation
    Vector3f compute_feedforward_acceleration(const Vector3f& velocity_target);
    Vector3f compute_feedforward_rates(const Quaternion& attitude_target);
    
    // Logging
    void log_controller_state();
};

#endif // VELOCITY_FEEDBACK_CONTROLLER_H