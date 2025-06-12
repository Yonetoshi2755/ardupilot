#ifndef UNIFIED_ESTIMATOR_H
#define UNIFIED_ESTIMATOR_H

#ifdef ARDUPILOT_BUILD
#include <AP_Math/AP_Math.h>
#else
#include "ap_math_compat.h"
#endif

/*
 * Unified System Identification and State Estimation
 * 
 * Augmented state vector (24 elements):
 * - Quaternion (4): attitude
 * - Angular velocity (3): body rates  
 * - Position (3): NED position
 * - Velocity (3): NED velocity
 * - Gyro bias (3): sensor bias
 * - Inertia diagonal (3): Ixx, Iyy, Izz
 * - Thrust coefficient (1): kT
 * - Torque coefficient (1): kQ
 * - Mass (1): vehicle mass
 * - Drag coefficients (2): linear, quadratic
 * 
 * This unified approach simultaneously estimates states and parameters,
 * correctly propagating uncertainty between them.
 */

class UnifiedEstimator {
public:
    UnifiedEstimator();
    
    // State indices
    static const uint8_t IDX_QUAT = 0;      // 0-3
    static const uint8_t IDX_OMEGA = 4;     // 4-6  
    static const uint8_t IDX_POS = 7;       // 7-9
    static const uint8_t IDX_VEL = 10;      // 10-12
    static const uint8_t IDX_BIAS = 13;     // 13-15
    static const uint8_t IDX_INERTIA = 16;  // 16-18 (Ixx, Iyy, Izz)
    static const uint8_t IDX_THRUST = 19;   // 19
    static const uint8_t IDX_TORQUE = 20;   // 20
    static const uint8_t IDX_MASS = 21;     // 21
    static const uint8_t IDX_DRAG = 22;     // 22-23
    
    static const uint8_t STATE_DIM = 16;
    static const uint8_t PARAM_DIM = 8;
    static const uint8_t AUG_DIM = 24;
    
    // Initialize
    void init(const Vector3f& init_pos, const Quaternion& init_att);
    
    // Main estimation step
    void update(const Vector3f& gyro, const Vector3f& accel, 
                const float motors[6], float dt);
    
    // Measurement updates
    void update_magnetometer(const Vector3f& mag);
    void update_gps(const Vector3f& pos, const Vector3f& vel);
    void update_baro(float altitude);
    
    // Get estimates
    Quaternion get_attitude() const;
    Vector3f get_position() const { return Vector3f(_x[IDX_POS], _x[IDX_POS+1], _x[IDX_POS+2]); }
    Vector3f get_velocity() const { return Vector3f(_x[IDX_VEL], _x[IDX_VEL+1], _x[IDX_VEL+2]); }
    Vector3f get_angular_velocity() const { return Vector3f(_x[IDX_OMEGA], _x[IDX_OMEGA+1], _x[IDX_OMEGA+2]); }
    Vector3f get_gyro_bias() const { return Vector3f(_x[IDX_BIAS], _x[IDX_BIAS+1], _x[IDX_BIAS+2]); }
    
    // Get parameters with uncertainty
    void get_inertia(Vector3f& inertia, Vector3f& std_dev) const;
    void get_thrust_coeff(float& coeff, float& std_dev) const;
    void get_mass(float& mass, float& std_dev) const;
    
    // Excitation for identifiability
    bool needs_excitation() const;
    Vector3f get_excitation_command() const;
    
    // Configuration
    void set_parameter_drift_rate(float drift_rate);
    void set_initial_parameter_uncertainty(float uncertainty);
    
private:
    // Augmented state and covariance
    float _x[AUG_DIM];
    float _P[AUG_DIM][AUG_DIM];
    
    // Process noise
    float _Q_state[STATE_DIM];
    float _Q_param[PARAM_DIM];
    
    // Measurement noise
    float _R_gyro, _R_accel, _R_mag, _R_gps_pos, _R_gps_vel, _R_baro;
    
    // Reference values
    Vector3f _gravity_ref;
    Vector3f _mag_ref;
    
    // Estimation mode
    bool _aggressive_id_mode;
    uint32_t _last_excitation_ms;
    
    // Numerical stability
    void enforce_constraints();
    void symmetrize_covariance();
    
    // Prediction step
    void predict(const Vector3f& gyro, const Vector3f& accel, 
                 const float motors[6], float dt);
    
    // Jacobian computation for augmented state
    void compute_augmented_jacobian(float F[AUG_DIM][AUG_DIM],
                                   const Vector3f& gyro,
                                   const Vector3f& accel,
                                   const float motors[6],
                                   float dt);
    
    // Motor model with parameters
    Vector3f compute_body_thrust(const float motors[6]) const;
    Vector3f compute_body_torque(const float motors[6]) const;
    
    // Information metrics
    float compute_parameter_observability(uint8_t param_idx) const;
    float compute_fisher_information() const;
    
    // Adaptive process noise
    void update_process_noise();
    
    // Quaternion operations
    void quaternion_normalize();
    static Quaternion quaternion_multiply(const Quaternion& q1, const Quaternion& q2);
    
    // Measurement update implementations
    void update_accel_measurement(const Vector3f& accel, const float motors[6]);
    
    // Helper functions for Jacobian computation
    float compute_thrust_derivative(const float motors[6], int axis) const;
};

// Implementation of information-theoretic control
class ExcitationController {
public:
    ExcitationController(UnifiedEstimator* estimator) : _estimator(estimator), _excitation_start_ms(0) {}
    
    // Generate optimal excitation signal
    Vector3f generate_excitation(float amplitude, float duration);
    
    // Persistence of excitation check
    bool is_sufficiently_excited() const;
    
private:
    UnifiedEstimator* _estimator;
    uint32_t _excitation_start_ms;
    
    // Fisher information maximization
    Vector3f optimize_for_identifiability();
};

#endif // UNIFIED_ESTIMATOR_H