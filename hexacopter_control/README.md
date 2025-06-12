# Hexacopter Advanced Control System

This package implements an advanced control system for hexacopter UAVs with system identification and EKF-based state estimation.

## Overview

The system consists of three main components:

1. **System Identification Module** - Online parameter estimation for hexacopter dynamics
2. **EKF Attitude/Velocity Estimator** - Extended Kalman Filter for state estimation
3. **Velocity Feedback Controller** - Cascaded control with adaptive gains

## Architecture

```
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  Sensors        │────▶│  EKF Estimator   │────▶│   Controller    │
│  (IMU,GPS,Mag) │     │  (Attitude/Vel)  │     │  (Velocity FB)  │
└─────────────────┘     └──────────────────┘     └────────┬────────┘
         │                       ▲                         │
         │                       │                         ▼
┌─────────────────┐     ┌──────────────────┐     ┌─────────────────┐
│  System ID      │────▶│ Identified Params│────▶│  Motor Mixer    │
│  (Online)       │     │  - Inertia       │     │                 │
└─────────────────┘     │  - Thrust/Torque │     └─────────────────┘
                        │  - Damping       │
                        └──────────────────┘
```

## Parameter Flow

The identified parameters from System ID flow through the system:

1. **System ID → EKF**: 
   - Inertia matrix improves angular dynamics prediction
   - Thrust coefficients enable control-aware state prediction
   - Process noise adapts based on parameter uncertainty

2. **System ID → Controller**:
   - Automatic gain scheduling based on identified inertia
   - Feed-forward terms use thrust/torque coefficients
   - Damping compensation in both velocity and rate loops

3. **EKF → Controller**:
   - State estimates with uncertainty quantification
   - Bias-compensated angular rates
   - Model-based prediction for reduced lag

## Components

### 1. System Identification (`hexacopter_sysid.h/cpp`)

Performs online identification of:
- Inertia matrix (Ixx, Iyy, Izz)
- Thrust and torque coefficients
- Aerodynamic damping coefficients
- Motor dynamics

**Test Sequences:**
- Step response
- Frequency sweep (chirp)
- Doublet maneuvers
- PRBS (Pseudo-Random Binary Sequence)

**Key Features:**
- Recursive Least Squares (RLS) estimation
- Forgetting factor for time-varying parameters
- Safety limits on test inputs
- Automatic data logging

### 2. EKF Attitude/Velocity Estimator (`ekf_attitude_velocity.h/cpp`)

**State Vector (16 elements):**
- Quaternion attitude (4)
- Angular velocity (3)
- Position NED (3)
- Velocity NED (3)
- Gyro bias (3)

**Measurements:**
- IMU (gyro + accelerometer)
- Magnetometer
- GPS position/velocity

**Key Features:**
- Quaternion-based attitude representation (singularity-free)
- Gyro bias estimation
- Adaptive measurement noise
- Symmetric covariance storage for efficiency

### 3. Velocity Feedback Controller (`velocity_feedback_controller.h/cpp`)

**Control Modes:**
- VELOCITY - Direct velocity control
- POSITION - Position hold with velocity inner loop
- STABILIZE - Attitude stabilization
- ACRO - Direct rate control

**Control Architecture:**
```
Position ──▶ Velocity ──▶ Attitude ──▶ Rate ──▶ Motors
   PID         PID          PID        PID
```

**Advanced Features:**
- Adaptive gain scheduling using SysID results
- Disturbance observer for wind rejection
- Reference model for smooth trajectories
- Anti-windup on integral terms
- Feed-forward compensation

## Usage

### Integration with ArduCopter

```cpp
// In ArduCopter vehicle class
class Copter {
    // Add new modules
    HexacopterSysID sysid;
    EKF_AttitudeVelocity ekf_custom;
    VelocityFeedbackController vel_controller;
    
    void init() {
        // Initialize modules
        sysid.init();
        ekf_custom.init(home_position, initial_attitude);
        vel_controller.init(&ekf_custom, &sysid);
    }
    
    void update() {
        // Update estimator
        ekf_custom.update_imu(gyro, accel, dt);
        ekf_custom.update_gps(gps_pos, gps_vel);
        
        // Run controller
        vel_controller.update(dt);
        
        // Get motor outputs
        float motors[6];
        vel_controller.get_motor_outputs(motors);
        
        // Send to motors
        for (int i = 0; i < 6; i++) {
            motors_output[i] = motors[i];
        }
    }
};
```

### Running System Identification

```cpp
// Start system identification
sysid.set_test_sequence(HexacopterSysID::TEST_FREQUENCY_SWEEP);
sysid.start_identification();

// System ID will run for configured duration
// Results are automatically logged and parameters updated
```

### Tuning Guide

1. **Initial Setup**
   - Set conservative gains in `velocity_feedback_controller.cpp`
   - Enable only attitude stabilization first
   - Verify sensor data quality

2. **System Identification**
   - Run step response tests in calm conditions
   - Start with small amplitude (5-10%)
   - Increase gradually while monitoring stability

3. **Controller Tuning**
   - Tune rate controller first (innermost loop)
   - Then attitude controller
   - Finally velocity and position controllers
   - Use logged data to analyze performance

4. **EKF Tuning**
   - Set realistic noise parameters based on sensor specs
   - Monitor innovation magnitudes
   - Adjust process noise if filter is too slow/fast

## Safety Considerations

1. **Test in SITL first** - Validate all changes in simulation
2. **Parameter limits** - Hard limits on tilt angles and rates
3. **Failsafe modes** - Automatic RTL on controller failure
4. **Gradual testing** - Start with small movements, increase gradually

## Performance Metrics

The controller logs extensive telemetry:
- State estimation accuracy
- Control errors and efforts  
- System identification progress
- Motor utilization

Monitor these metrics to assess performance and tune the system.

## Future Enhancements

1. **Nonlinear Control**
   - Sliding mode control for robustness
   - Model Predictive Control (MPC)
   - Backstepping control

2. **Advanced Estimation**
   - Unscented Kalman Filter (UKF)
   - Multi-sensor fusion
   - Visual-inertial odometry

3. **Fault Detection**
   - Motor failure detection
   - Sensor fault isolation
   - Reconfigurable control

## References

1. Mahony, R., Kumar, V., & Corke, P. (2012). Multirotor aerial vehicles: Modeling, estimation, and control of quadrotor.
2. Pounds, P. (2007). Design and control of a quad-rotor aircraft.
3. Ljung, L. (1999). System identification: Theory for the user.