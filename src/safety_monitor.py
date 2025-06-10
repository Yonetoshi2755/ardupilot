#!/usr/bin/env python3
"""
Safety Monitoring Module for Bridge Inspector
Monitors vehicle health, environmental conditions, and mission safety
"""

import asyncio
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
import logging
import math
from pymavlink import mavutil


class SafetyStatus(Enum):
    """Safety status levels"""
    SAFE = "safe"
    CAUTION = "caution"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


class SafetyAction(Enum):
    """Actions to take based on safety conditions"""
    CONTINUE = "continue"
    SLOW_DOWN = "slow_down"
    HOVER = "hover"
    RTL = "rtl"
    LAND = "land"
    EMERGENCY_STOP = "emergency_stop"


@dataclass
class SafetyThresholds:
    """Configurable safety thresholds"""
    # Battery
    battery_warning: float = 40  # percent
    battery_critical: float = 30
    battery_emergency: float = 20
    voltage_min_cell: float = 3.3  # volts per cell
    
    # GPS
    gps_satellites_min: int = 8
    gps_hdop_max: float = 2.5
    gps_position_error_max: float = 5.0  # meters
    
    # Wind and weather
    wind_speed_warning: float = 8  # m/s
    wind_speed_max: float = 12
    wind_gust_max: float = 15
    
    # Vibration
    vibration_warning: float = 30
    vibration_critical: float = 60
    
    # Temperature
    temp_min: float = -10  # Celsius
    temp_max: float = 45
    
    # Altitude
    altitude_max: float = 120  # meters
    altitude_min: float = 5
    terrain_min_clearance: float = 10
    
    # Distance
    geofence_warning: float = 400  # meters
    geofence_max: float = 500
    
    # Time
    mission_timeout: float = 3600  # seconds (1 hour)
    hover_timeout: float = 300  # seconds stuck in one place


@dataclass
class SafetyReport:
    """Safety assessment report"""
    timestamp: datetime
    overall_status: SafetyStatus
    recommended_action: SafetyAction
    issues: List[str]
    metrics: Dict[str, float]
    can_continue: bool


class SafetyMonitor:
    """Monitors safety conditions during inspection missions"""
    
    def __init__(self, vehicle_connection, thresholds: SafetyThresholds = None):
        self.vehicle = vehicle_connection
        self.thresholds = thresholds or SafetyThresholds()
        self.logger = logging.getLogger(__name__)
        
        # State tracking
        self.home_position = None
        self.mission_start_time = None
        self.last_position = None
        self.last_position_time = None
        self.position_history = []  # For stuck detection
        
        # Metrics history for trend analysis
        self.battery_history = []
        self.vibration_history = []
        self.gps_history = []
        
        # Safety flags
        self.emergency_declared = False
        self.rtl_triggered = False
        
    def start_monitoring(self, home_position: Tuple[float, float]):
        """Initialize monitoring for a new mission"""
        self.home_position = home_position
        self.mission_start_time = time.time()
        self.position_history = []
        self.emergency_declared = False
        self.rtl_triggered = False
        
        self.logger.info(f"Safety monitoring started. Home: {home_position}")
        
    async def continuous_monitor(self, callback=None):
        """Run continuous safety monitoring"""
        while not self.emergency_declared:
            try:
                report = await self.assess_safety()
                
                # Log status changes
                if report.overall_status != SafetyStatus.SAFE:
                    self.logger.warning(
                        f"Safety status: {report.overall_status.value} - "
                        f"Action: {report.recommended_action.value}"
                    )
                    for issue in report.issues:
                        self.logger.warning(f"  Issue: {issue}")
                
                # Execute callback if provided
                if callback:
                    await callback(report)
                
                # Take automatic action if critical
                if report.overall_status == SafetyStatus.EMERGENCY:
                    await self.execute_emergency_procedure(report.recommended_action)
                
                await asyncio.sleep(1)  # Check every second
                
            except Exception as e:
                self.logger.error(f"Safety monitoring error: {e}")
                await asyncio.sleep(1)
    
    async def assess_safety(self) -> SafetyReport:
        """Perform comprehensive safety assessment"""
        issues = []
        metrics = {}
        
        # Get current telemetry
        battery_status = self._check_battery()
        gps_status = self._check_gps()
        position_status = self._check_position()
        vibration_status = self._check_vibration()
        environment_status = self._check_environment()
        time_status = self._check_mission_time()
        
        # Aggregate metrics
        metrics.update(battery_status['metrics'])
        metrics.update(gps_status['metrics'])
        metrics.update(position_status['metrics'])
        metrics.update(vibration_status['metrics'])
        metrics.update(environment_status['metrics'])
        metrics.update(time_status['metrics'])
        
        # Collect issues
        all_statuses = [
            battery_status, gps_status, position_status,
            vibration_status, environment_status, time_status
        ]
        
        for status in all_statuses:
            issues.extend(status['issues'])
        
        # Determine overall status
        status_levels = [s['status'] for s in all_statuses]
        overall_status = self._get_worst_status(status_levels)
        
        # Determine recommended action
        action = self._determine_action(overall_status, all_statuses)
        
        # Check if mission can continue
        can_continue = overall_status in [SafetyStatus.SAFE, SafetyStatus.CAUTION]
        
        report = SafetyReport(
            timestamp=datetime.now(),
            overall_status=overall_status,
            recommended_action=action,
            issues=issues,
            metrics=metrics,
            can_continue=can_continue
        )
        
        return report
    
    def _check_battery(self) -> Dict:
        """Check battery status"""
        issues = []
        metrics = {}
        status = SafetyStatus.SAFE
        
        # Get battery info
        msg = self.vehicle.recv_match(type='SYS_STATUS', blocking=False)
        if msg:
            battery_percent = msg.battery_remaining
            voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != -1 else 0
            current = msg.current_battery / 100.0 if msg.current_battery != -1 else 0
            
            metrics['battery_percent'] = battery_percent
            metrics['battery_voltage'] = voltage
            metrics['battery_current'] = current
            
            # Check battery percentage
            if battery_percent < self.thresholds.battery_emergency:
                status = SafetyStatus.EMERGENCY
                issues.append(f"EMERGENCY: Battery critical at {battery_percent}%")
            elif battery_percent < self.thresholds.battery_critical:
                status = SafetyStatus.CRITICAL
                issues.append(f"Battery very low: {battery_percent}%")
            elif battery_percent < self.thresholds.battery_warning:
                status = SafetyStatus.WARNING
                issues.append(f"Battery low: {battery_percent}%")
            
            # Check voltage (assuming 4S battery)
            if voltage > 0:
                cell_voltage = voltage / 4
                metrics['battery_cell_voltage'] = cell_voltage
                
                if cell_voltage < self.thresholds.voltage_min_cell:
                    status = self._get_worst_status([status, SafetyStatus.WARNING])
                    issues.append(f"Low cell voltage: {cell_voltage:.2f}V")
            
            # Add to history for trend analysis
            self.battery_history.append({
                'time': time.time(),
                'percent': battery_percent,
                'voltage': voltage
            })
            
            # Keep only last 60 seconds
            self.battery_history = [
                h for h in self.battery_history 
                if time.time() - h['time'] < 60
            ]
            
            # Check discharge rate
            if len(self.battery_history) > 10:
                time_span = self.battery_history[-1]['time'] - self.battery_history[0]['time']
                percent_drop = self.battery_history[0]['percent'] - self.battery_history[-1]['percent']
                
                if time_span > 0:
                    discharge_rate = percent_drop / time_span * 60  # percent per minute
                    metrics['battery_discharge_rate'] = discharge_rate
                    
                    if discharge_rate > 2.0:  # More than 2% per minute
                        status = self._get_worst_status([status, SafetyStatus.WARNING])
                        issues.append(f"High discharge rate: {discharge_rate:.1f}%/min")
        
        return {
            'status': status,
            'issues': issues,
            'metrics': metrics
        }
    
    def _check_gps(self) -> Dict:
        """Check GPS status"""
        issues = []
        metrics = {}
        status = SafetyStatus.SAFE
        
        # Get GPS info
        msg = self.vehicle.recv_match(type='GPS_RAW_INT', blocking=False)
        if msg:
            satellites = msg.satellites_visible
            hdop = msg.eph / 100.0 if msg.eph != 65535 else 99
            fix_type = msg.fix_type
            
            metrics['gps_satellites'] = satellites
            metrics['gps_hdop'] = hdop
            metrics['gps_fix_type'] = fix_type
            
            # Check fix type
            if fix_type < 3:  # No 3D fix
                status = SafetyStatus.CRITICAL
                issues.append("No GPS 3D fix")
            
            # Check satellites
            if satellites < self.thresholds.gps_satellites_min:
                status = self._get_worst_status([status, SafetyStatus.WARNING])
                issues.append(f"Low GPS satellites: {satellites}")
            
            # Check HDOP
            if hdop > self.thresholds.gps_hdop_max:
                status = self._get_worst_status([status, SafetyStatus.WARNING])
                issues.append(f"Poor GPS accuracy: HDOP {hdop:.1f}")
            
            # Add to history
            self.gps_history.append({
                'time': time.time(),
                'satellites': satellites,
                'hdop': hdop
            })
            
            # Keep only last 30 seconds
            self.gps_history = [
                h for h in self.gps_history 
                if time.time() - h['time'] < 30
            ]
        
        return {
            'status': status,
            'issues': issues,
            'metrics': metrics
        }
    
    def _check_position(self) -> Dict:
        """Check position and geofence"""
        issues = []
        metrics = {}
        status = SafetyStatus.SAFE
        
        # Get current position
        msg = self.vehicle.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if msg and self.home_position:
            current_lat = msg.lat / 1e7
            current_lon = msg.lon / 1e7
            current_alt = msg.relative_alt / 1000.0
            
            # Calculate distance from home
            distance_from_home = self._calculate_distance(
                current_lat, current_lon,
                self.home_position[0], self.home_position[1]
            )
            
            metrics['distance_from_home'] = distance_from_home
            metrics['altitude'] = current_alt
            
            # Check geofence
            if distance_from_home > self.thresholds.geofence_max:
                status = SafetyStatus.CRITICAL
                issues.append(f"Geofence violation: {distance_from_home:.0f}m from home")
            elif distance_from_home > self.thresholds.geofence_warning:
                status = SafetyStatus.WARNING
                issues.append(f"Approaching geofence: {distance_from_home:.0f}m from home")
            
            # Check altitude limits
            if current_alt > self.thresholds.altitude_max:
                status = self._get_worst_status([status, SafetyStatus.WARNING])
                issues.append(f"Altitude too high: {current_alt:.1f}m")
            elif current_alt < self.thresholds.altitude_min:
                status = self._get_worst_status([status, SafetyStatus.WARNING])
                issues.append(f"Altitude too low: {current_alt:.1f}m")
            
            # Check if stuck (not moving)
            current_pos = (current_lat, current_lon, current_alt)
            current_time = time.time()
            
            if self.last_position and self.last_position_time:
                time_diff = current_time - self.last_position_time
                pos_diff = self._calculate_3d_distance(current_pos, self.last_position)
                
                if time_diff > 10 and pos_diff < 1.0:  # Less than 1m movement in 10s
                    # Add to position history
                    self.position_history.append({
                        'time': current_time,
                        'position': current_pos
                    })
                    
                    # Check if stuck for too long
                    stuck_duration = current_time - self.position_history[0]['time']
                    if stuck_duration > self.thresholds.hover_timeout:
                        status = self._get_worst_status([status, SafetyStatus.WARNING])
                        issues.append(f"Vehicle stuck for {stuck_duration:.0f}s")
                else:
                    # Moving, clear history
                    self.position_history = []
            
            self.last_position = current_pos
            self.last_position_time = current_time
        
        return {
            'status': status,
            'issues': issues,
            'metrics': metrics
        }
    
    def _check_vibration(self) -> Dict:
        """Check vibration levels"""
        issues = []
        metrics = {}
        status = SafetyStatus.SAFE
        
        # Get vibration data
        msg = self.vehicle.recv_match(type='VIBRATION', blocking=False)
        if msg:
            vibe_x = msg.vibration_x
            vibe_y = msg.vibration_y
            vibe_z = msg.vibration_z
            
            # Calculate total vibration
            total_vibe = math.sqrt(vibe_x**2 + vibe_y**2 + vibe_z**2)
            
            metrics['vibration_total'] = total_vibe
            metrics['vibration_x'] = vibe_x
            metrics['vibration_y'] = vibe_y
            metrics['vibration_z'] = vibe_z
            
            # Check thresholds
            if total_vibe > self.thresholds.vibration_critical:
                status = SafetyStatus.CRITICAL
                issues.append(f"Critical vibration: {total_vibe:.1f}")
            elif total_vibe > self.thresholds.vibration_warning:
                status = SafetyStatus.WARNING
                issues.append(f"High vibration: {total_vibe:.1f}")
            
            # Add to history
            self.vibration_history.append({
                'time': time.time(),
                'vibration': total_vibe
            })
            
            # Keep only last 30 seconds
            self.vibration_history = [
                h for h in self.vibration_history 
                if time.time() - h['time'] < 30
            ]
            
            # Check for increasing trend
            if len(self.vibration_history) > 10:
                recent_avg = sum(h['vibration'] for h in self.vibration_history[-5:]) / 5
                older_avg = sum(h['vibration'] for h in self.vibration_history[:5]) / 5
                
                if recent_avg > older_avg * 1.5:  # 50% increase
                    status = self._get_worst_status([status, SafetyStatus.CAUTION])
                    issues.append("Vibration increasing")
        
        return {
            'status': status,
            'issues': issues,
            'metrics': metrics
        }
    
    def _check_environment(self) -> Dict:
        """Check environmental conditions"""
        issues = []
        metrics = {}
        status = SafetyStatus.SAFE
        
        # Get wind estimate (if available)
        msg = self.vehicle.recv_match(type='WIND', blocking=False)
        if msg:
            wind_speed = math.sqrt(msg.speed_x**2 + msg.speed_y**2)
            wind_z = msg.speed_z
            
            metrics['wind_speed'] = wind_speed
            metrics['wind_vertical'] = wind_z
            
            # Check wind speed
            if wind_speed > self.thresholds.wind_speed_max:
                status = SafetyStatus.CRITICAL
                issues.append(f"Wind too strong: {wind_speed:.1f}m/s")
            elif wind_speed > self.thresholds.wind_speed_warning:
                status = SafetyStatus.WARNING
                issues.append(f"High wind: {wind_speed:.1f}m/s")
        
        # Get temperature (if available)
        msg = self.vehicle.recv_match(type='SCALED_PRESSURE', blocking=False)
        if msg:
            temperature = msg.temperature / 100.0  # Convert to Celsius
            metrics['temperature'] = temperature
            
            # Check temperature
            if temperature < self.thresholds.temp_min:
                status = self._get_worst_status([status, SafetyStatus.WARNING])
                issues.append(f"Temperature too low: {temperature:.1f}°C")
            elif temperature > self.thresholds.temp_max:
                status = self._get_worst_status([status, SafetyStatus.WARNING])
                issues.append(f"Temperature too high: {temperature:.1f}°C")
        
        return {
            'status': status,
            'issues': issues,
            'metrics': metrics
        }
    
    def _check_mission_time(self) -> Dict:
        """Check mission duration"""
        issues = []
        metrics = {}
        status = SafetyStatus.SAFE
        
        if self.mission_start_time:
            mission_duration = time.time() - self.mission_start_time
            metrics['mission_duration'] = mission_duration
            
            # Check timeout
            if mission_duration > self.thresholds.mission_timeout:
                status = SafetyStatus.WARNING
                issues.append(f"Mission timeout: {mission_duration/60:.0f} minutes")
        
        return {
            'status': status,
            'issues': issues,
            'metrics': metrics
        }
    
    def _get_worst_status(self, statuses: List[SafetyStatus]) -> SafetyStatus:
        """Return the worst status from a list"""
        priority = {
            SafetyStatus.SAFE: 0,
            SafetyStatus.CAUTION: 1,
            SafetyStatus.WARNING: 2,
            SafetyStatus.CRITICAL: 3,
            SafetyStatus.EMERGENCY: 4
        }
        
        return max(statuses, key=lambda s: priority.get(s, 0))
    
    def _determine_action(self, overall_status: SafetyStatus, 
                         all_statuses: List[Dict]) -> SafetyAction:
        """Determine recommended action based on status"""
        
        # Emergency actions
        if overall_status == SafetyStatus.EMERGENCY:
            # Check specific emergency conditions
            battery_status = next(s for s in all_statuses if 'battery_percent' in s.get('metrics', {}))
            if battery_status['metrics'].get('battery_percent', 100) < self.thresholds.battery_emergency:
                return SafetyAction.LAND  # Land immediately for critical battery
            return SafetyAction.EMERGENCY_STOP
        
        # Critical actions
        if overall_status == SafetyStatus.CRITICAL:
            # Check specific critical conditions
            position_status = next(s for s in all_statuses if 'distance_from_home' in s.get('metrics', {}))
            if position_status['metrics'].get('distance_from_home', 0) > self.thresholds.geofence_max:
                return SafetyAction.RTL  # Return for geofence violation
            return SafetyAction.RTL
        
        # Warning actions
        if overall_status == SafetyStatus.WARNING:
            return SafetyAction.SLOW_DOWN
        
        # Caution actions
        if overall_status == SafetyStatus.CAUTION:
            return SafetyAction.CONTINUE
        
        return SafetyAction.CONTINUE
    
    async def execute_emergency_procedure(self, action: SafetyAction):
        """Execute emergency safety action"""
        self.logger.critical(f"Executing emergency action: {action.value}")
        self.emergency_declared = True
        
        if action == SafetyAction.LAND:
            # Set LAND mode
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                9  # LAND mode
            )
        elif action == SafetyAction.RTL:
            # Set RTL mode
            self.rtl_triggered = True
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                6  # RTL mode
            )
        elif action == SafetyAction.EMERGENCY_STOP:
            # Disarm immediately
            self.vehicle.mav.command_long_send(
                self.vehicle.target_system,
                self.vehicle.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                0, 21196, 0, 0, 0, 0, 0  # Force disarm
            )
        elif action == SafetyAction.HOVER:
            # Set LOITER/HOVER mode
            self.vehicle.mav.set_mode_send(
                self.vehicle.target_system,
                mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                5  # LOITER mode
            )
    
    def _calculate_distance(self, lat1: float, lon1: float, 
                          lat2: float, lon2: float) -> float:
        """Calculate distance between two GPS coordinates"""
        dlat = (lat2 - lat1) * 111320
        dlon = (lon2 - lon1) * 111320 * math.cos(math.radians(lat1))
        return math.sqrt(dlat**2 + dlon**2)
    
    def _calculate_3d_distance(self, pos1: Tuple, pos2: Tuple) -> float:
        """Calculate 3D distance between positions"""
        dlat = (pos2[0] - pos1[0]) * 111320
        dlon = (pos2[1] - pos1[1]) * 111320 * math.cos(math.radians(pos1[0]))
        dalt = pos2[2] - pos1[2]
        return math.sqrt(dlat**2 + dlon**2 + dalt**2)
    
    def get_safety_summary(self) -> Dict:
        """Get current safety metrics summary"""
        summary = {
            'monitoring_active': self.mission_start_time is not None,
            'emergency_declared': self.emergency_declared,
            'rtl_triggered': self.rtl_triggered,
            'mission_duration': time.time() - self.mission_start_time if self.mission_start_time else 0
        }
        
        # Add latest metrics
        if self.battery_history:
            summary['battery_percent'] = self.battery_history[-1]['percent']
        
        if self.gps_history:
            summary['gps_satellites'] = self.gps_history[-1]['satellites']
        
        if self.vibration_history:
            summary['vibration_level'] = self.vibration_history[-1]['vibration']
        
        return summary


async def example_safety_callback(report: SafetyReport):
    """Example callback for safety monitoring"""
    if report.overall_status != SafetyStatus.SAFE:
        print(f"\nSafety Alert: {report.overall_status.value}")
        print(f"Recommended Action: {report.recommended_action.value}")
        print("Issues:")
        for issue in report.issues:
            print(f"  - {issue}")
        print(f"Can Continue: {report.can_continue}")


def main():
    """Test safety monitor"""
    import sys
    
    # Configure logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Create mock vehicle connection
    vehicle = mavutil.mavlink_connection('udp:127.0.0.1:14550')
    
    # Create safety monitor
    monitor = SafetyMonitor(vehicle)
    
    # Start monitoring
    home = (37.7749, -122.4194)  # San Francisco
    monitor.start_monitoring(home)
    
    # Run monitoring with callback
    try:
        asyncio.run(monitor.continuous_monitor(example_safety_callback))
    except KeyboardInterrupt:
        print("\nSafety monitoring stopped")
        summary = monitor.get_safety_summary()
        print(f"Final summary: {summary}")


if __name__ == "__main__":
    main()