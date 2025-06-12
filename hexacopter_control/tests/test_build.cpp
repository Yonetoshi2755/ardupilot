/*
 * Simple build test for hexacopter control system
 * Verifies that all headers compile correctly
 */

#include <stdio.h>

// Test includes
#include "hexacopter_sysid.h"
#include "ekf_attitude_velocity.h"
#include "velocity_feedback_controller.h"

int main()
{
    printf("Hexacopter Control System - Build Test\n");
    printf("=====================================\n");
    
    // Test instantiation of classes
    printf("Creating HexacopterSysID...\n");
    HexacopterSysID sysid;
    
    printf("Creating EKF_AttitudeVelocity...\n");
    EKF_AttitudeVelocity ekf;
    
    printf("Creating VelocityFeedbackController...\n");
    VelocityFeedbackController controller;
    
    // Test basic initialization
    printf("\nInitializing components...\n");
    sysid.init();
    ekf.init(Vector3f(0, 0, 0), Quaternion(1, 0, 0, 0));
    controller.init(&ekf, &sysid);
    
    // Test some basic operations
    printf("\nTesting basic operations...\n");
    
    // EKF update
    Vector3f gyro(0.1f, 0.0f, 0.0f);
    Vector3f accel(0.0f, 0.0f, -9.81f);
    ekf.update_imu(gyro, accel, 0.01f);
    
    // Controller update
    controller.set_velocity_target(Vector3f(1.0f, 0.0f, 0.0f));
    controller.update(0.01f);
    
    // Get outputs
    float motors[6];
    controller.get_motor_outputs(motors);
    
    printf("\nMotor outputs: ");
    for (int i = 0; i < 6; i++) {
        printf("%.3f ", motors[i]);
    }
    printf("\n");
    
    // Check state estimates
    Vector3f euler = ekf.get_euler_angles();
    printf("\nEKF Euler angles: Roll=%.3f, Pitch=%.3f, Yaw=%.3f\n",
           degrees(euler.x), degrees(euler.y), degrees(euler.z));
    
    Vector3f velocity = ekf.get_velocity();
    printf("EKF Velocity: X=%.3f, Y=%.3f, Z=%.3f\n",
           velocity.x, velocity.y, velocity.z);
    
    // Test system ID
    printf("\nSystem ID parameters:\n");
    printf("Thrust coefficient: %.6f\n", sysid.get_thrust_coefficient());
    printf("Torque coefficient: %.6f\n", sysid.get_torque_coefficient());
    
    Matrix3f inertia = sysid.get_inertia_matrix();
    printf("Inertia matrix diagonal: [%.4f, %.4f, %.4f]\n",
           inertia.a.x, inertia.b.y, inertia.c.z);
    
    printf("\nBuild test completed successfully!\n");
    
    return 0;
}