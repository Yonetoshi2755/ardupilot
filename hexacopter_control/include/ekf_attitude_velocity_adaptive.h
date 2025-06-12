#ifndef EKF_ATTITUDE_VELOCITY_ADAPTIVE_H
#define EKF_ATTITUDE_VELOCITY_ADAPTIVE_H

#include "ekf_attitude_velocity.h"
#include "hexacopter_sysid.h"

/*
 * Adaptive EKF that uses System ID results
 * 
 * This extends the base EKF to incorporate:
 * - Identified inertia matrix for better angular acceleration prediction
 * - Thrust/torque coefficients for control input modeling
 * - Damping coefficients for drag compensation
 * - Adaptive process noise based on parameter uncertainty
 */

class EKF_AttitudeVelocityAdaptive : public EKF_AttitudeVelocity {
public:
    EKF_AttitudeVelocityAdaptive();
    
    // Set system ID reference
    void set_sysid(HexacopterSysID* sysid) { _sysid = sysid; }
    
    // Override predict to use identified parameters
    void predict_with_controls(const Vector3f& gyro, const Vector3f& accel, 
                              const float motor_commands[6], float dt);
    
    // Update process noise based on SysID confidence
    void update_adaptive_noise();
    
private:
    HexacopterSysID* _sysid;
    
    // Extended state for control-aware prediction
    void predict_with_motor_model(const float motor_commands[6], float dt);
    
    // Compute expected accelerations from motor commands
    Vector3f compute_thrust_acceleration(const float motors[6]) const;
    Vector3f compute_torque_rates(const float motors[6]) const;
    
    // Adaptive Jacobian computation
    void compute_F_jacobian_adaptive(float F[STATE_SIZE][STATE_SIZE],
                                     const float* x,
                                     const Vector3f& gyro,
                                     const Vector3f& accel,
                                     const float motor_commands[6],
                                     float dt) const;
    
    // Covariance propagation
    void propagate_covariance(const float F[STATE_SIZE][STATE_SIZE], float dt);
    
    // Adaptive noise scaling
    float _inertia_uncertainty_scale;
    float _thrust_uncertainty_scale;
};

#endif // EKF_ATTITUDE_VELOCITY_ADAPTIVE_H