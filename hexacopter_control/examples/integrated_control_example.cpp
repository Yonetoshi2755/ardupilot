/*
 * Example showing integrated use of System ID, Adaptive EKF, and Controller
 * Demonstrates how identified parameters flow through the system
 */

#include <cstdio>
#include "hexacopter_sysid.h"
#include "ekf_attitude_velocity_adaptive.h"
#include "velocity_feedback_controller.h"

class IntegratedHexacopterControl {
public:
    IntegratedHexacopterControl() {
        // Initialize all components
        sysid.init();
        ekf.init(Vector3f(0, 0, 0), Quaternion(1, 0, 0, 0));
        controller.init(&ekf, &sysid);
        
        // Link adaptive EKF to system ID
        ekf.set_sysid(&sysid);
    }
    
    void update(float dt) {
        // Get sensor data (from actual sensors in real implementation)
        Vector3f gyro = get_gyro_measurement();
        Vector3f accel = get_accel_measurement();
        Vector3f mag = get_mag_measurement();
        
        // Get current motor commands from previous iteration
        float motors[6];
        controller.get_motor_outputs(motors);
        
        // Step 1: Update System ID if enabled
        if (sysid_enabled && !controller_active) {
            sysid.update(dt);
            
            // System ID generates its own test commands
            if (sysid.is_running()) {
                // Use system ID test commands instead of controller
                // This would set motors directly for system ID tests
                return;
            }
        }
        
        // Step 2: Run adaptive EKF with motor model
        ekf.predict_with_controls(gyro, accel, motors, dt);
        
        // Update with measurements
        ekf.update_mag(mag);
        if (gps_available) {
            ekf.update_gps(gps_position, gps_velocity);
        }
        
        // Step 3: Run controller with adaptive gains
        if (controller_active) {
            controller.update(dt);
            
            // Controller automatically uses latest SysID parameters
            // through update_adaptive_gains() when sysid.is_running()
        }
        
        // Log integrated state
        log_system_state();
    }
    
    void start_system_identification() {
        sysid_enabled = true;
        controller_active = false;
        sysid.set_test_sequence(HexacopterSysID::TEST_FREQUENCY_SWEEP);
        sysid.start_identification();
    }
    
    void start_flight_control() {
        sysid_enabled = false;
        controller_active = true;
        
        // Ensure we have good parameters
        if (sysid.get_sample_count() < 100) {
            printf("Warning: Limited system ID data available\n");
        }
        
        printf("Using identified parameters:\n");
        printf("  Thrust coeff: %.2e N/(rad/s)^2\n", sysid.get_thrust_coefficient());
        printf("  Torque coeff: %.2e Nm/(rad/s)^2\n", sysid.get_torque_coefficient());
        
        Matrix3f inertia = sysid.get_inertia_matrix();
        printf("  Inertia: [%.4f, %.4f, %.4f] kg*m^2\n", 
               inertia.a.x, inertia.b.y, inertia.c.z);
    }
    
private:
    HexacopterSysID sysid;
    EKF_AttitudeVelocityAdaptive ekf;
    VelocityFeedbackController controller;
    
    bool sysid_enabled = false;
    bool controller_active = true;
    bool gps_available = false;
    
    Vector3f gps_position;
    Vector3f gps_velocity;
    
    // Placeholder sensor functions
    Vector3f get_gyro_measurement() { return Vector3f(0, 0, 0); }
    Vector3f get_accel_measurement() { return Vector3f(0, 0, -GRAVITY_MSS); }
    Vector3f get_mag_measurement() { return Vector3f(0.2f, 0, 0.4f); }
    
    void log_system_state() {
        // Log key parameters showing integration
        static uint32_t log_counter = 0;
        if (++log_counter % 100 == 0) {
            printf("=== Integrated System State ===\n");
            
            // EKF state
            Vector3f euler = ekf.get_euler_angles();
            Vector3f velocity = ekf.get_velocity();
            printf("EKF: Roll=%.1f° Pitch=%.1f° Yaw=%.1f°\n",
                   degrees(euler.x), degrees(euler.y), degrees(euler.z));
            printf("     Vel=[%.2f, %.2f, %.2f] m/s\n",
                   velocity.x, velocity.y, velocity.z);
            
            // Show how EKF uncertainty adapts during SysID
            printf("     Att_var=%.4f Vel_var=%.4f\n",
                   ekf.get_attitude_variance(),
                   ekf.get_velocity_variance());
            
            // Controller gains (showing adaptation)
            const auto& gains = controller.get_gains();
            printf("Controller Gains (adaptive):\n");
            printf("     Rate_P=[%.3f, %.3f, %.3f]\n",
                   gains.rate_p.x, gains.rate_p.y, gains.rate_p.z);
            printf("     Rate_D=[%.3f, %.3f, %.3f]\n",
                   gains.rate_d.x, gains.rate_d.y, gains.rate_d.z);
            
            // Performance metrics
            const auto& perf = controller.get_performance();
            printf("Performance: Vel_err=%.3f m/s, Att_err=%.3f rad\n",
                   perf.velocity_error_mag, perf.attitude_error_mag);
        }
    }
};

// Example usage showing the parameter flow
int main() {
    IntegratedHexacopterControl hexacopter;
    
    printf("=== Hexacopter Integrated Control Demo ===\n\n");
    
    // Phase 1: System Identification
    printf("Phase 1: Starting System Identification...\n");
    hexacopter.start_system_identification();
    
    // Run system ID for 20 seconds
    for (int i = 0; i < 2000; i++) {
        hexacopter.update(0.01f);  // 100Hz update
    }
    
    // Phase 2: Flight Control with Identified Parameters
    printf("\nPhase 2: Starting Flight Control...\n");
    hexacopter.start_flight_control();
    
    // Run flight control
    for (int i = 0; i < 1000; i++) {
        hexacopter.update(0.01f);
    }
    
    printf("\nDemo complete - System ID parameters successfully integrated!\n");
    
    return 0;
}