#ifndef EKF_ATTITUDE_VELOCITY_H
#define EKF_ATTITUDE_VELOCITY_H

#ifdef ARDUPILOT_BUILD
#include <AP_Math/AP_Math.h>
#include <AP_Math/quaternion.h>
#include <AP_Math/matrix_alg.h>
#else
#include "ap_math_compat.h"
#endif

/*
 * Extended Kalman Filter for Attitude and Velocity Estimation
 * 
 * State vector (16 elements):
 * - Quaternion (4): attitude representation
 * - Angular velocity (3): body frame angular rates
 * - Position (3): NED frame position
 * - Velocity (3): NED frame velocity
 * - Gyro bias (3): gyroscope bias estimation
 * 
 * Measurements:
 * - Gyroscope (3): angular rates
 * - Accelerometer (3): specific force
 * - Magnetometer (3): magnetic field
 * - GPS position (3): position measurement
 * - GPS velocity (3): velocity measurement
 */

class EKF_AttitudeVelocity {
public:
    EKF_AttitudeVelocity();

    // Initialize filter
    void init(const Vector3f& init_position, const Quaternion& init_attitude);

    // Main update functions
    void predict(float dt);
    void update_imu(const Vector3f& gyro, const Vector3f& accel, float dt);
    void update_mag(const Vector3f& mag);
    void update_gps(const Vector3f& position, const Vector3f& velocity);

    // Get state estimates
    Quaternion get_quaternion() const { return Quaternion(_state[0], _state[1], _state[2], _state[3]); }
    Vector3f get_euler_angles() const;
    Vector3f get_angular_velocity() const { return Vector3f(_state[4], _state[5], _state[6]); }
    Vector3f get_position() const { return Vector3f(_state[7], _state[8], _state[9]); }
    Vector3f get_velocity() const { return Vector3f(_state[10], _state[11], _state[12]); }
    Vector3f get_gyro_bias() const { return Vector3f(_state[13], _state[14], _state[15]); }

    // Get covariance
    float get_attitude_variance() const;
    float get_velocity_variance() const;
    float get_position_variance() const;

    // Configuration
    void set_process_noise(float gyro_noise, float accel_noise, float bias_noise);
    void set_measurement_noise(float gyro_noise, float accel_noise, float mag_noise, float gps_pos_noise, float gps_vel_noise);

protected:
    static const uint8_t STATE_SIZE = 16;
    static const uint8_t QUAT_IDX = 0;
    static const uint8_t OMEGA_IDX = 4;
    static const uint8_t POS_IDX = 7;
    static const uint8_t VEL_IDX = 10;
    static const uint8_t BIAS_IDX = 13;

    // State vector
    float _state[STATE_SIZE];

    // Covariance matrix (using compressed storage for symmetric matrix)
    float _P[(STATE_SIZE * (STATE_SIZE + 1)) / 2];

    // Process noise parameters
    float _gyro_noise_variance;
    float _accel_noise_variance;
    float _bias_noise_variance;

    // Measurement noise parameters
    float _gyro_meas_variance;
    float _accel_meas_variance;
    float _mag_meas_variance;
    float _gps_pos_variance;
    float _gps_vel_variance;

    // Reference vectors
    Vector3f _mag_earth;    // Earth magnetic field reference
    Vector3f _gravity_ref;  // Gravity reference vector

    // Helper functions
    void normalize_quaternion();
    
    // Covariance matrix access (symmetric storage)
    float& P(uint8_t i, uint8_t j);
    float P(uint8_t i, uint8_t j) const;
    uint16_t P_index(uint8_t i, uint8_t j) const;

    // Quaternion operations
    Quaternion quaternion_multiply(const Quaternion& q1, const Quaternion& q2) const;

    // State transition functions
    void f_state_transition(float* x_new, const float* x, const Vector3f& gyro, const Vector3f& accel, float dt) const;
    
    // Jacobian computation
    void compute_F_jacobian(float F[STATE_SIZE][STATE_SIZE], const float* x, const Vector3f& gyro, const Vector3f& accel, float dt) const;
    void compute_H_accel_jacobian(float H[3][STATE_SIZE], const float* x) const;
    void compute_H_mag_jacobian(float H[3][STATE_SIZE], const float* x) const;

private:
    Matrix24 compute_F_matrix(float dt) const;
    void predict_state(float dt);
    void predict_covariance(const Matrix24& F, float dt);
    
    // Measurement update helpers
    void update_with_measurement(const VectorN<float, 3>& z, 
                                const VectorN<float, 3>& h,
                                const MatrixN<float, 3>& H,
                                const MatrixN<float, 3>& R);

    Matrix4 quaternion_to_dcm_jacobian(const Quaternion& q) const;
    
    // Measurement functions
    Vector3f h_accel(const float* x) const;
    Vector3f h_mag(const float* x) const;
};

// Inline covariance matrix access
inline float& EKF_AttitudeVelocity::P(uint8_t i, uint8_t j) {
    return _P[P_index(i, j)];
}

inline float EKF_AttitudeVelocity::P(uint8_t i, uint8_t j) const {
    return _P[P_index(i, j)];
}

inline uint16_t EKF_AttitudeVelocity::P_index(uint8_t i, uint8_t j) const {
    if (i > j) {
        uint8_t tmp = i;
        i = j;
        j = tmp;
    }
    return (j * (j + 1)) / 2 + i;
}

#endif // EKF_ATTITUDE_VELOCITY_H