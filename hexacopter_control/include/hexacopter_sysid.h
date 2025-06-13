#ifndef HEXACOPTER_SYSID_H
#define HEXACOPTER_SYSID_H

#ifdef ARDUPILOT_BUILD
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Motors/AP_Motors.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Param/AP_Param.h>
#else
#include "ap_math_compat.h"
#endif

/*
 * System Identification for Hexacopter
 * This module performs online system identification for a 6-motor hexacopter
 * to determine dynamic parameters including:
 * - Thrust coefficients
 * - Moment arms
 * - Inertia matrix
 * - Aerodynamic damping coefficients
 */

class HexacopterSysID {
public:
    HexacopterSysID();

    // Parameters
    static const struct AP_Param::GroupInfo var_info[];

    // Initialize system identification
    void init();

    // Run one iteration of system identification
    void update(float dt);

    // Start/stop data collection
    void start_identification();
    void stop_identification();
    bool is_running() const { return _running; }

    // Get identified parameters
    const Matrix3f& get_inertia_matrix() const { return _inertia_matrix; }
    const Vector3f& get_damping_coefficients() const { return _damping_coeff; }
    float get_thrust_coefficient() const { return _thrust_coeff; }
    float get_torque_coefficient() const { return _torque_coeff; }
    float get_vehicle_mass() const { return _vehicle_mass; }
    
    // Compute thrust to vertical acceleration/velocity
    float compute_thrust_acceleration(float total_thrust) const;
    Vector3f compute_expected_acceleration(const float motor_commands[6], const Quaternion& attitude) const;
    
    // Get estimation quality metrics
    uint32_t get_sample_count() const { return _sample_count; }
    float get_fitness_score() const { return _fitness_score; }

    // Motor test sequences
    enum TestSequence {
        TEST_NONE = 0,
        TEST_STEP_RESPONSE,
        TEST_FREQUENCY_SWEEP,
        TEST_DOUBLET,
        TEST_PRBS  // Pseudo-Random Binary Sequence
    };

    // Set test sequence
    void set_test_sequence(TestSequence seq) { _test_sequence = seq; }

private:
    // Configuration
#ifdef ARDUPILOT_BUILD
    AP_Int8 _enable;
    AP_Float _test_throttle_min;
    AP_Float _test_throttle_max;
    AP_Float _test_duration;
    AP_Int8 _test_axis;  // 0=roll, 1=pitch, 2=yaw
    AP_Float _vehicle_mass;  // Vehicle mass in kg
#else
    int8_t _enable;
    float _test_throttle_min;
    float _test_throttle_max;
    float _test_duration;
    int8_t _test_axis;  // 0=roll, 1=pitch, 2=yaw
    float _vehicle_mass;  // Vehicle mass in kg
#endif

    // State
    bool _running;
    uint32_t _start_time_ms;
    TestSequence _test_sequence;

    // Data collection buffers
    static const uint16_t MAX_SAMPLES = 1000;
    struct Sample {
        uint32_t timestamp_ms;
        Vector3f angular_rate;
        Vector3f angular_accel;
        Vector3f motor_commands;
        float throttle;
        Vector3f acceleration;  // Body frame acceleration
        float vertical_accel;   // World frame vertical acceleration
    };
    Sample _samples[MAX_SAMPLES];
    uint16_t _sample_count;

    // Identified parameters
    Matrix3f _inertia_matrix;
    Vector3f _damping_coeff;
    float _thrust_coeff;
    float _torque_coeff;
    Vector3f _moment_arms[6];  // For each motor
    float _fitness_score;       // Quality metric for identification

    // Motor mixing matrix for hexacopter
    // Layout: X configuration
    //     1   2
    //   6   X   3
    //     5   4
    static const float _motor_angles[6];
    static const float _motor_arm_length;

    // Methods
    void collect_sample(const Vector3f& gyro, const Vector3f& motor_cmd, float throttle);
    void generate_test_signal(float& roll_out, float& pitch_out, float& yaw_out, float& throttle_out);
    void perform_identification();
    
    // Recursive Least Squares (RLS) for online parameter estimation
    void rls_update(const Vector3f& input, const Vector3f& output);
    Matrix3f _P_matrix;  // Covariance matrix for RLS
    float _forgetting_factor;

    // Test signal generators
    float generate_step(float time, float magnitude);
    float generate_sweep(float time, float f_start, float f_end, float magnitude);
    float generate_doublet(float time, float width, float magnitude);
    float generate_prbs(float time, float bit_time, float magnitude);

    // Logging
    void log_identification_data();
};

#endif // HEXACOPTER_SYSID_H