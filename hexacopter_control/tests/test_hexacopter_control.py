#!/usr/bin/env python3
"""
Test script for hexacopter control system
Simulates the control system with synthetic sensor data
"""

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time

class HexacopterSimulator:
    def __init__(self):
        # Physical parameters
        self.mass = 2.0  # kg
        self.arm_length = 0.25  # m
        self.inertia = np.diag([0.02, 0.02, 0.04])  # kg*m^2
        
        # State: [pos, vel, quat, omega]
        self.state = np.zeros(13)
        self.state[6] = 1.0  # quaternion w component
        
        # Motor configuration (X hexacopter)
        self.motor_angles = np.array([60, 120, 180, 240, 300, 0]) * np.pi / 180
        self.motor_directions = np.array([1, -1, 1, -1, 1, -1])  # Yaw direction
        
        # Aerodynamic coefficients
        self.thrust_coeff = 1.5e-5
        self.torque_coeff = 3.0e-7
        self.drag_coeff = 0.1
        
        # Control system components (simplified Python versions)
        self.ekf_state = {
            'position': np.zeros(3),
            'velocity': np.zeros(3),
            'quaternion': np.array([1, 0, 0, 0]),
            'angular_velocity': np.zeros(3),
            'gyro_bias': np.zeros(3)
        }
        
        self.controller_gains = {
            'vel_p': np.array([5.0, 5.0, 10.0]),
            'vel_i': np.array([1.0, 1.0, 3.0]),
            'att_p': np.array([6.0, 6.0, 4.0]),
            'rate_p': np.array([0.15, 0.15, 0.1])
        }
        
        self.integral_errors = {
            'velocity': np.zeros(3),
            'attitude': np.zeros(3)
        }
        
        # Data logging
        self.time_history = []
        self.state_history = []
        self.motor_history = []
        self.setpoint_history = []
        
    def quaternion_to_rotation_matrix(self, q):
        """Convert quaternion to rotation matrix"""
        w, x, y, z = q
        return np.array([
            [1-2*(y**2+z**2), 2*(x*y-w*z), 2*(x*z+w*y)],
            [2*(x*y+w*z), 1-2*(x**2+z**2), 2*(y*z-w*x)],
            [2*(x*z-w*y), 2*(y*z+w*x), 1-2*(x**2+y**2)]
        ])
        
    def quaternion_derivative(self, q, omega):
        """Compute quaternion derivative from angular velocity"""
        w, x, y, z = q
        p, q_rate, r = omega
        
        q_dot = 0.5 * np.array([
            -x*p - y*q_rate - z*r,
            w*p + y*r - z*q_rate,
            w*q_rate - x*r + z*p,
            w*r + x*q_rate - y*p
        ])
        return q_dot
        
    def dynamics(self, state, motor_commands, dt):
        """Hexacopter dynamics model"""
        pos = state[0:3]
        vel = state[3:6]
        quat = state[6:10]
        omega = state[10:13]
        
        # Normalize quaternion
        quat = quat / np.linalg.norm(quat)
        
        # Rotation matrix
        R = self.quaternion_to_rotation_matrix(quat)
        
        # Total thrust and torques from motors
        thrust_total = 0
        torques = np.zeros(3)
        
        for i in range(6):
            thrust_i = self.thrust_coeff * motor_commands[i]**2
            thrust_total += thrust_i
            
            # Torque contributions
            arm_vector = self.arm_length * np.array([
                np.cos(self.motor_angles[i]),
                np.sin(self.motor_angles[i]),
                0
            ])
            
            # Thrust torque
            thrust_torque = np.cross(arm_vector, np.array([0, 0, thrust_i]))
            torques += thrust_torque
            
            # Yaw torque
            torques[2] += self.torque_coeff * motor_commands[i]**2 * self.motor_directions[i]
        
        # Forces in body frame
        thrust_body = np.array([0, 0, -thrust_total])
        
        # Forces in world frame
        thrust_world = R @ thrust_body
        gravity = np.array([0, 0, self.mass * 9.81])
        drag = -self.drag_coeff * vel
        
        # Accelerations
        acc = (thrust_world + gravity + drag) / self.mass
        angular_acc = np.linalg.inv(self.inertia) @ (torques - np.cross(omega, self.inertia @ omega))
        
        # State derivatives
        pos_dot = vel
        vel_dot = acc
        quat_dot = self.quaternion_derivative(quat, omega)
        omega_dot = angular_acc
        
        # Update state
        new_state = np.zeros(13)
        new_state[0:3] = pos + pos_dot * dt
        new_state[3:6] = vel + vel_dot * dt
        new_state[6:10] = quat + quat_dot * dt
        new_state[10:13] = omega + omega_dot * dt
        
        # Normalize quaternion
        new_state[6:10] = new_state[6:10] / np.linalg.norm(new_state[6:10])
        
        return new_state
        
    def ekf_update(self, gyro, accel, dt):
        """Simplified EKF update"""
        # Predict
        self.ekf_state['position'] += self.ekf_state['velocity'] * dt
        
        # Update angular velocity (with bias compensation)
        self.ekf_state['angular_velocity'] = gyro - self.ekf_state['gyro_bias']
        
        # Update quaternion
        q_dot = self.quaternion_derivative(self.ekf_state['quaternion'], 
                                         self.ekf_state['angular_velocity'])
        self.ekf_state['quaternion'] += q_dot * dt
        self.ekf_state['quaternion'] /= np.linalg.norm(self.ekf_state['quaternion'])
        
        # Accelerometer update (simplified)
        R = self.quaternion_to_rotation_matrix(self.ekf_state['quaternion'])
        expected_accel = R.T @ np.array([0, 0, -9.81])
        accel_error = accel - expected_accel
        
        # Simple complementary filter for attitude correction
        correction = 0.01 * accel_error
        self.ekf_state['angular_velocity'] += correction
        
    def velocity_controller(self, velocity_target, dt):
        """Velocity feedback controller"""
        # Velocity error
        vel_error = velocity_target - self.ekf_state['velocity']
        
        # PID control
        acc_cmd = self.controller_gains['vel_p'] * vel_error
        self.integral_errors['velocity'] += vel_error * dt
        acc_cmd += self.controller_gains['vel_i'] * self.integral_errors['velocity']
        
        # Convert to attitude setpoint (simplified)
        # Assume small angles
        pitch_sp = -acc_cmd[0] / 9.81
        roll_sp = acc_cmd[1] / 9.81
        
        # Limit tilt
        max_tilt = 0.785  # 45 degrees
        pitch_sp = np.clip(pitch_sp, -max_tilt, max_tilt)
        roll_sp = np.clip(roll_sp, -max_tilt, max_tilt)
        
        return np.array([roll_sp, pitch_sp, 0])  # Roll, pitch, yaw
        
    def attitude_controller(self, attitude_target):
        """Attitude controller"""
        # Convert target euler to quaternion (simplified)
        roll, pitch, yaw = attitude_target
        
        # Current attitude (euler angles from quaternion)
        R = self.quaternion_to_rotation_matrix(self.ekf_state['quaternion'])
        current_pitch = np.arcsin(-R[2, 0])
        current_roll = np.arctan2(R[2, 1], R[2, 2])
        current_yaw = np.arctan2(R[1, 0], R[0, 0])
        
        # Attitude error
        att_error = np.array([
            roll - current_roll,
            pitch - current_pitch,
            yaw - current_yaw
        ])
        
        # Wrap yaw error
        att_error[2] = np.arctan2(np.sin(att_error[2]), np.cos(att_error[2]))
        
        # Rate setpoint
        rate_sp = self.controller_gains['att_p'] * att_error
        
        return rate_sp
        
    def rate_controller(self, rate_target):
        """Rate controller"""
        rate_error = rate_target - self.ekf_state['angular_velocity']
        torque_cmd = self.controller_gains['rate_p'] * rate_error
        
        # Mix to motors
        motor_commands = self.mix_motors(torque_cmd)
        
        return motor_commands
        
    def mix_motors(self, torque_cmd):
        """Motor mixing for hexacopter"""
        # Base throttle (hover)
        hover_throttle = np.sqrt(self.mass * 9.81 / (6 * self.thrust_coeff))
        motors = np.ones(6) * hover_throttle
        
        # Add differential thrust for torques
        for i in range(6):
            # Roll torque
            motors[i] += torque_cmd[0] * np.sin(self.motor_angles[i]) / (3 * self.arm_length)
            
            # Pitch torque
            motors[i] += torque_cmd[1] * np.cos(self.motor_angles[i]) / (3 * self.arm_length)
            
            # Yaw torque
            motors[i] += torque_cmd[2] * self.motor_directions[i] / 6
            
        # Normalize and limit
        motors = np.clip(motors, 0, 1000)
        
        return motors
        
    def simulate_step(self, velocity_target, dt):
        """Run one simulation step"""
        # Get sensor measurements (with noise)
        gyro = self.state[10:13] + np.random.normal(0, 0.01, 3)
        R = self.quaternion_to_rotation_matrix(self.state[6:10])
        accel = R.T @ np.array([0, 0, -9.81]) + np.random.normal(0, 0.1, 3)
        
        # EKF update
        self.ekf_update(gyro, accel, dt)
        
        # Control cascade
        attitude_target = self.velocity_controller(velocity_target, dt)
        rate_target = self.attitude_controller(attitude_target)
        motor_commands = self.rate_controller(rate_target)
        
        # Update dynamics
        self.state = self.dynamics(self.state, motor_commands, dt)
        
        # Update EKF with true state (simulated GPS)
        if np.random.rand() < 0.1:  # 10 Hz GPS
            self.ekf_state['position'] = self.state[0:3] + np.random.normal(0, 0.1, 3)
            self.ekf_state['velocity'] = self.state[3:6] + np.random.normal(0, 0.05, 3)
            
        # Log data
        self.time_history.append(len(self.time_history) * dt)
        self.state_history.append(self.state.copy())
        self.motor_history.append(motor_commands.copy())
        self.setpoint_history.append(velocity_target.copy())
        
    def plot_results(self):
        """Plot simulation results"""
        time = np.array(self.time_history)
        states = np.array(self.state_history)
        motors = np.array(self.motor_history)
        setpoints = np.array(self.setpoint_history)
        
        fig, axes = plt.subplots(3, 2, figsize=(12, 10))
        
        # Position
        ax = axes[0, 0]
        ax.plot(time, states[:, 0], label='X')
        ax.plot(time, states[:, 1], label='Y')
        ax.plot(time, states[:, 2], label='Z')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (m)')
        ax.legend()
        ax.grid(True)
        
        # Velocity
        ax = axes[0, 1]
        ax.plot(time, states[:, 3], label='Vx')
        ax.plot(time, states[:, 4], label='Vy')
        ax.plot(time, states[:, 5], label='Vz')
        ax.plot(time, setpoints[:, 0], '--', label='Vx_sp')
        ax.plot(time, setpoints[:, 1], '--', label='Vy_sp')
        ax.plot(time, setpoints[:, 2], '--', label='Vz_sp')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (m/s)')
        ax.legend()
        ax.grid(True)
        
        # Attitude (Euler angles)
        ax = axes[1, 0]
        euler_angles = []
        for state in states:
            R = self.quaternion_to_rotation_matrix(state[6:10])
            pitch = np.arcsin(-R[2, 0])
            roll = np.arctan2(R[2, 1], R[2, 2])
            yaw = np.arctan2(R[1, 0], R[0, 0])
            euler_angles.append([roll, pitch, yaw])
        euler_angles = np.array(euler_angles)
        
        ax.plot(time, np.degrees(euler_angles[:, 0]), label='Roll')
        ax.plot(time, np.degrees(euler_angles[:, 1]), label='Pitch')
        ax.plot(time, np.degrees(euler_angles[:, 2]), label='Yaw')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Euler Angles (deg)')
        ax.legend()
        ax.grid(True)
        
        # Angular rates
        ax = axes[1, 1]
        ax.plot(time, states[:, 10], label='p')
        ax.plot(time, states[:, 11], label='q')
        ax.plot(time, states[:, 12], label='r')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Angular Rate (rad/s)')
        ax.legend()
        ax.grid(True)
        
        # Motor commands
        ax = axes[2, 0]
        for i in range(6):
            ax.plot(time, motors[:, i], label=f'Motor {i+1}')
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Motor Command')
        ax.legend()
        ax.grid(True)
        
        # 3D trajectory
        ax = fig.add_subplot(3, 2, 6, projection='3d')
        ax.plot(states[:, 0], states[:, 1], -states[:, 2])
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Altitude (m)')
        ax.set_title('3D Trajectory')
        
        plt.tight_layout()
        plt.show()

def run_velocity_tracking_test():
    """Test velocity tracking performance"""
    print("Running velocity tracking test...")
    
    sim = HexacopterSimulator()
    dt = 0.01  # 100 Hz
    
    # Velocity setpoint sequence
    velocity_sequence = [
        (0, np.array([0, 0, -1])),     # Climb
        (5, np.array([2, 0, 0])),      # Forward
        (10, np.array([0, 2, 0])),     # Right
        (15, np.array([-2, 0, 0])),    # Backward
        (20, np.array([0, -2, 0])),    # Left
        (25, np.array([0, 0, 0])),     # Hover
        (30, np.array([0, 0, 1]))      # Descend
    ]
    
    current_setpoint = np.zeros(3)
    seq_index = 0
    
    # Run simulation
    for i in range(3500):  # 35 seconds
        t = i * dt
        
        # Update velocity setpoint
        if seq_index < len(velocity_sequence) - 1:
            if t >= velocity_sequence[seq_index + 1][0]:
                seq_index += 1
                current_setpoint = velocity_sequence[seq_index][1]
        
        sim.simulate_step(current_setpoint, dt)
        
        if i % 100 == 0:
            print(f"Time: {t:.1f}s, Pos: {sim.state[0:3]}, Vel: {sim.state[3:6]}")
    
    # Plot results
    sim.plot_results()
    
    # Calculate performance metrics
    states = np.array(sim.state_history)
    setpoints = np.array(sim.setpoint_history)
    
    # Velocity tracking error
    vel_error = states[:, 3:6] - setpoints
    rmse_vel = np.sqrt(np.mean(vel_error**2, axis=0))
    
    print(f"\nPerformance Metrics:")
    print(f"Velocity RMSE: X={rmse_vel[0]:.3f}, Y={rmse_vel[1]:.3f}, Z={rmse_vel[2]:.3f} m/s")
    
    # Settling time analysis
    steady_state_threshold = 0.1  # 10% of setpoint
    print(f"\nSettling times (to within {steady_state_threshold*100}% of setpoint):")
    
    for i, (t_switch, vel_sp) in enumerate(velocity_sequence[1:], 1):
        # Find when velocity settles
        start_idx = int(t_switch / dt)
        for j in range(start_idx, len(states)):
            vel_error_mag = np.linalg.norm(states[j, 3:6] - vel_sp)
            vel_sp_mag = np.linalg.norm(vel_sp)
            if vel_sp_mag > 0:
                if vel_error_mag / vel_sp_mag < steady_state_threshold:
                    settling_time = (j - start_idx) * dt
                    print(f"  Transition {i}: {settling_time:.2f}s")
                    break

if __name__ == "__main__":
    run_velocity_tracking_test()