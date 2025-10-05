"""
Realistic UAV Flight Data Generator (Clean Implementation)

Generates drone flight paths based on actual UAV physics and common mission patterns.
Focus: Simplicity, realism, and efficiency.

Key features:
- Smooth curved paths using Catmull-Rom splines (no sharp turns)
- Physics-based motion with proper turn radius
- Realistic mission patterns (patrol, survey, delivery, etc.)
"""

import math
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
from datetime import datetime
import json
from pathlib import Path
from dataclasses import dataclass
from scipy.interpolate import CubicSpline

from .flight_plan import FlightPlan, Waypoint


@dataclass
class DronePhysics:
    """Physical constraints for realistic drone flight."""
    cruise_speed: float = 15.0      # m/s (typical for DJI-style drones)
    min_speed: float = 3.0          # m/s (minimum forward speed)
    max_speed: float = 20.0         # m/s (max cruise speed)
    max_acceleration: float = 2.0   # m/s² (gentle acceleration)
    max_turn_rate: float = 30.0     # deg/s (turn rate at cruise speed)
    min_turn_radius: float = 50.0   # m (at cruise speed)
    max_climb_rate: float = 3.0     # m/s (vertical speed)
    max_bank_angle: float = 25.0    # degrees (for coordinated turns)
    
    def get_turn_radius(self, speed: float) -> float:
        """Calculate turn radius for given speed (coordinated turn)."""
        # R = v²/(g·tan(bank_angle))
        g = 9.81
        bank_rad = math.radians(self.max_bank_angle)
        radius = (speed ** 2) / (g * math.tan(bank_rad))
        return max(radius, self.min_turn_radius)


class FlightPattern:
    """Generate different mission patterns for drones."""
    
    def __init__(self, airspace: Tuple[float, float, float], physics: DronePhysics, rng: np.random.Generator):
        self.airspace = airspace  # (width, length, height)
        self.physics = physics
        self.rng = rng
    
    def point_to_point(self, alt_min: float, alt_max: float) -> List[Tuple[float, float, float]]:
        """Simple A to B mission with optional intermediate waypoint."""
        margin = 500  # Stay away from edges
        
        # Start point
        start = (
            float(self.rng.uniform(margin, self.airspace[0] - margin)),
            float(self.rng.uniform(margin, self.airspace[1] - margin)),
            float(self.rng.uniform(alt_min, alt_max))
        )
        
        # End point (far from start)
        min_distance = 2000
        max_attempts = 10
        for _ in range(max_attempts):
            end = (
                float(self.rng.uniform(margin, self.airspace[0] - margin)),
                float(self.rng.uniform(margin, self.airspace[1] - margin)),
                float(self.rng.uniform(alt_min, alt_max))
            )
            dist = math.sqrt((end[0]-start[0])**2 + (end[1]-start[1])**2)
            if dist >= min_distance:
                break
        
        # Maybe add intermediate waypoint (50% chance)
        if self.rng.random() < 0.5:
            mid = (
                (start[0] + end[0]) / 2 + float(self.rng.uniform(-800, 800)),
                (start[1] + end[1]) / 2 + float(self.rng.uniform(-800, 800)),
                float(self.rng.uniform(alt_min, alt_max))
            )
            return [start, mid, end]
        
        return [start, end]
    
    def patrol_route(self, alt_min: float, alt_max: float) -> List[Tuple[float, float, float]]:
        """Rectangular or circular patrol pattern."""
        if self.rng.random() < 0.5:
            return self._rectangular_patrol(alt_min, alt_max)
        else:
            return self._circular_patrol(alt_min, alt_max)
    
    def _rectangular_patrol(self, alt_min: float, alt_max: float) -> List[Tuple[float, float, float]]:
        """Rectangular patrol pattern."""
        margin = 800
        center_x = self.airspace[0] / 2 + float(self.rng.uniform(-1500, 1500))
        center_y = self.airspace[1] / 2 + float(self.rng.uniform(-1500, 1500))
        
        width = float(self.rng.uniform(1000, 2500))
        height = float(self.rng.uniform(1000, 2500))
        altitude = float(self.rng.uniform(alt_min, alt_max))
        
        # Ensure within bounds
        half_w, half_h = width/2, height/2
        center_x = np.clip(center_x, margin + half_w, self.airspace[0] - margin - half_w)
        center_y = np.clip(center_y, margin + half_h, self.airspace[1] - margin - half_h)
        
        # Four corners
        corners = [
            (center_x - half_w, center_y - half_h, altitude),
            (center_x + half_w, center_y - half_h, altitude),
            (center_x + half_w, center_y + half_h, altitude),
            (center_x - half_w, center_y + half_h, altitude),
            (center_x - half_w, center_y - half_h, altitude)  # Close the loop
        ]
        
        return corners
    
    def _circular_patrol(self, alt_min: float, alt_max: float) -> List[Tuple[float, float, float]]:
        """Circular patrol pattern (approximated with polygon)."""
        margin = 800
        center_x = self.airspace[0] / 2 + float(self.rng.uniform(-1500, 1500))
        center_y = self.airspace[1] / 2 + float(self.rng.uniform(-1500, 1500))
        
        radius = float(self.rng.uniform(600, 1500))
        altitude = float(self.rng.uniform(alt_min, alt_max))
        
        # Ensure circle stays within bounds
        max_radius = min(
            center_x - margin,
            self.airspace[0] - center_x - margin,
            center_y - margin,
            self.airspace[1] - center_y - margin
        )
        radius = min(radius, max_radius)
        
        # Create circle with 6-8 points
        num_points = int(self.rng.integers(6, 9))
        points = []
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            points.append((x, y, altitude))
        
        points.append(points[0])  # Close the loop
        return points
    
    def survey_grid(self, alt_min: float, alt_max: float) -> List[Tuple[float, float, float]]:
        """Survey/mapping grid pattern (lawnmower)."""
        margin = 1000
        
        # Define survey area
        x_start = float(self.rng.uniform(margin, self.airspace[0] / 2))
        x_end = float(self.rng.uniform(self.airspace[0] / 2, self.airspace[0] - margin))
        y_start = float(self.rng.uniform(margin, self.airspace[1] / 2))
        y_end = float(self.rng.uniform(self.airspace[1] / 2, self.airspace[1] - margin))
        
        altitude = float(self.rng.uniform(alt_min, alt_max))
        
        # Number of passes
        num_passes = int(self.rng.integers(3, 6))
        y_positions = np.linspace(y_start, y_end, num_passes)
        
        # Create lawnmower pattern
        points = []
        for i, y in enumerate(y_positions):
            if i % 2 == 0:
                points.append((x_start, float(y), altitude))
                points.append((x_end, float(y), altitude))
            else:
                points.append((x_end, float(y), altitude))
                points.append((x_start, float(y), altitude))
        
        return points
    
    def waypoint_tour(self, alt_min: float, alt_max: float) -> List[Tuple[float, float, float]]:
        """Visit multiple random waypoints (delivery/inspection)."""
        margin = 600
        num_waypoints = int(self.rng.integers(3, 7))
        
        points = []
        for _ in range(num_waypoints):
            x = float(self.rng.uniform(margin, self.airspace[0] - margin))
            y = float(self.rng.uniform(margin, self.airspace[1] - margin))
            z = float(self.rng.uniform(alt_min, alt_max))
            points.append((x, y, z))
        
        return points


class TrajectoryGenerator:
    """Generate smooth, curved trajectories from waypoints using splines."""
    
    def __init__(self, physics: DronePhysics, airspace_bounds: Optional[Tuple[float, float, float]] = None):
        self.physics = physics
        self.airspace_bounds = airspace_bounds  # (max_x, max_y, max_z)
    
    def generate_trajectory(
        self, 
        control_points: List[Tuple[float, float, float]],
        start_time: float = 0.0
    ) -> List[Waypoint]:
        """
        Generate smooth curved trajectory through control points.
        
        Uses cubic spline interpolation for smooth, flowing curves
        with no sharp turns - just like a real drone would fly.
        """
        if len(control_points) < 2:
            raise ValueError("Need at least 2 control points")
        
        # For 2 points, use simple interpolation
        if len(control_points) == 2:
            return self._generate_simple_segment(
                control_points[0], control_points[1], start_time
            )
        
        # For 3+ points, use smooth spline interpolation
        return self._generate_spline_trajectory(control_points, start_time)
    
    def _generate_simple_segment(
        self,
        start: Tuple[float, float, float],
        end: Tuple[float, float, float],
        start_time: float
    ) -> List[Waypoint]:
        """Generate simple point-to-point segment with smooth motion."""
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        dz = end[2] - start[2]
        
        horizontal_dist = math.sqrt(dx**2 + dy**2)
        total_dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if total_dist < 1.0:
            return [Waypoint(start[0], start[1], start[2], start_time)]
        
        # Duration based on cruise speed
        segment_duration = horizontal_dist / self.physics.cruise_speed
        num_samples = max(2, int(segment_duration / 18.0) + 1)
        
        waypoints = []
        for i in range(num_samples):
            t_ratio = i / (num_samples - 1)
            
            # Smooth ease-in-out
            s = 3 * t_ratio**2 - 2 * t_ratio**3
            
            x = start[0] + dx * s
            y = start[1] + dy * s
            z = start[2] + dz * s
            t = start_time + segment_duration * t_ratio
            
            waypoints.append(Waypoint(x, y, z, t))
        
        return waypoints
    
    def _generate_spline_trajectory(
        self,
        control_points: List[Tuple[float, float, float]],
        start_time: float
    ) -> List[Waypoint]:
        """
        Generate smooth curved trajectory using cubic splines.
        
        This creates flowing, natural curves through waypoints with no sharp turns.
        """
        # Extract coordinates
        xs = np.array([p[0] for p in control_points])
        ys = np.array([p[1] for p in control_points])
        zs = np.array([p[2] for p in control_points])
        
        # Calculate cumulative arc length as parameter
        # This ensures smooth speed profile
        segments = len(control_points) - 1
        distances = [0.0]
        
        for i in range(segments):
            dx = xs[i+1] - xs[i]
            dy = ys[i+1] - ys[i]
            dz = zs[i+1] - zs[i]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
            distances.append(distances[-1] + dist)
        
        distances = np.array(distances)
        total_distance = distances[-1]
        
        if total_distance < 1.0:
            return [Waypoint(xs[0], ys[0], zs[0], start_time)]
        
        # Create smooth splines for each dimension
        # Use 'natural' boundary conditions for smooth start/end
        spline_x = CubicSpline(distances, xs, bc_type='natural')
        spline_y = CubicSpline(distances, ys, bc_type='natural')
        spline_z = CubicSpline(distances, zs, bc_type='natural')
        
        # Calculate total flight time
        total_time = total_distance / self.physics.cruise_speed
        
        # Sample the spline at regular time intervals
        # This gives us smooth, evenly-timed waypoints
        num_samples = max(3, int(total_time / 18.0) + 1)
        
        waypoints = []
        for i in range(num_samples):
            # Linear time interpolation
            t_ratio = i / (num_samples - 1)
            
            # Map time to distance along spline (with ease-in-out for smooth accel)
            s_ratio = 3 * t_ratio**2 - 2 * t_ratio**3
            s = s_ratio * total_distance
            
            # Evaluate spline at this distance
            x = float(spline_x(s))
            y = float(spline_y(s))
            z = float(spline_z(s))
            
            # Clamp to prevent spline overshoot
            # Splines can overshoot slightly beyond control points
            z = float(np.clip(z, min(zs), max(zs)))
            
            # Also clamp to airspace bounds if available
            if self.airspace_bounds:
                x = float(np.clip(x, 0, self.airspace_bounds[0]))
                y = float(np.clip(y, 0, self.airspace_bounds[1]))
            
            t = start_time + total_time * t_ratio
            
            waypoints.append(Waypoint(x, y, z, t))
        
        return waypoints


class FlightDataGenerator:
    """
    Generate realistic UAV flight data with physics-based motion.
    
    Simpler, cleaner implementation focusing on:
    - Realistic flight patterns (patrol, survey, point-to-point, etc.)
    - Physics-based smooth trajectories
    - Efficient representation (10-30 waypoints per flight)
    """
    
    def __init__(
        self,
        airspace_size: Tuple[float, float, float] = (10000, 10000, 500),
        min_altitude: float = 50.0,
        max_altitude: float = 400.0,
        seed: Optional[int] = None,
        physics: Optional[DronePhysics] = None
    ):
        """
        Initialize the flight data generator.
        
        Args:
            airspace_size: (width, length, height) in meters
            min_altitude: Minimum flight altitude in meters
            max_altitude: Maximum flight altitude in meters
            seed: Random seed for reproducibility
            physics: DronePhysics configuration (uses defaults if None)
        """
        self.airspace_size = airspace_size
        self.min_altitude = min_altitude
        self.max_altitude = max_altitude
        self.physics = physics or DronePhysics()
        
        self.rng = np.random.default_rng(seed)
        self.pattern_gen = FlightPattern(airspace_size, self.physics, self.rng)
        self.trajectory_gen = TrajectoryGenerator(self.physics, airspace_size)
    
    def generate_random_waypoint(
        self,
        time: float,
        z_range: Optional[Tuple[float, float]] = None
    ) -> Waypoint:
        """
        Generate a random waypoint within airspace bounds.
        
        Args:
            time: Time value for the waypoint
            z_range: Optional altitude range override
            
        Returns:
            Random waypoint
        """
        x = float(self.rng.uniform(0, self.airspace_size[0]))
        y = float(self.rng.uniform(0, self.airspace_size[1]))
        
        if z_range:
            z = float(self.rng.uniform(z_range[0], z_range[1]))
        else:
            z = float(self.rng.uniform(self.min_altitude, self.max_altitude))
        
        return Waypoint(x=x, y=y, z=z, time=time)
    
    def generate_flight(
        self,
        uav_id: str,
        pattern_type: Optional[str] = None,
        priority: Optional[int] = None
    ) -> FlightPlan:
        """
        Generate a single UAV flight with realistic pattern.
        
        Args:
            uav_id: UAV identifier
            pattern_type: Type of mission pattern:
                - 'point_to_point' or 'straight': Simple A to B
                - 'patrol': Rectangular or circular patrol
                - 'survey': Grid survey pattern
                - 'waypoint_tour' or 'multi_waypoint': Visit multiple waypoints
                - None: Random selection
            priority: Flight priority (random 1-5 if None)
        
        Returns:
            FlightPlan with realistic trajectory
        """
        # Choose pattern
        if pattern_type is None:
            pattern_type = self.rng.choice([
                'point_to_point',
                'patrol',
                'survey',
                'waypoint_tour'
            ])
        
        # Backward compatibility aliases
        if pattern_type == 'straight':
            pattern_type = 'point_to_point'
        elif pattern_type == 'multi_waypoint':
            pattern_type = 'waypoint_tour'
        
        # Generate control points based on pattern
        if pattern_type == 'point_to_point':
            control_points = self.pattern_gen.point_to_point(
                self.min_altitude, self.max_altitude
            )
        elif pattern_type == 'patrol':
            control_points = self.pattern_gen.patrol_route(
                self.min_altitude, self.max_altitude
            )
        elif pattern_type == 'survey':
            control_points = self.pattern_gen.survey_grid(
                self.min_altitude, self.max_altitude
            )
        elif pattern_type == 'waypoint_tour':
            control_points = self.pattern_gen.waypoint_tour(
                self.min_altitude, self.max_altitude
            )
        else:
            raise ValueError(f"Unknown pattern type: {pattern_type}")
        
        # Generate smooth trajectory through control points
        waypoints = self.trajectory_gen.generate_trajectory(
            control_points, start_time=0.0
        )
        
        # Random priority if not specified
        if priority is None:
            priority = int(self.rng.integers(1, 6))
        
        # Speed with some variation
        speed = float(self.physics.cruise_speed * self.rng.uniform(0.9, 1.1))
        
        return FlightPlan(
            uav_id=uav_id,
            waypoints=waypoints,
            speed=speed,
            priority=priority
        )
    
    def generate_straight_flight(
        self,
        uav_id: str,
        start_point: Optional[Waypoint] = None,
        end_point: Optional[Waypoint] = None,
        speed: float = 15.0
    ) -> FlightPlan:
        """
        Generate a simple straight-line flight between two points.
        
        Args:
            uav_id: UAV identifier
            start_point: Starting waypoint (random if None)
            end_point: Ending waypoint (random if None)
            speed: Flight speed in m/s
            
        Returns:
            Flight plan with straight path
        """
        if start_point is None:
            control_points = self.pattern_gen.point_to_point(
                self.min_altitude, self.max_altitude
            )
        else:
            if end_point is None:
                # Generate random endpoint
                margin = 500
                end = (
                    float(self.rng.uniform(margin, self.airspace_size[0] - margin)),
                    float(self.rng.uniform(margin, self.airspace_size[1] - margin)),
                    float(self.rng.uniform(self.min_altitude, self.max_altitude))
                )
            else:
                end = (end_point.x, end_point.y, end_point.z)
            
            control_points = [(start_point.x, start_point.y, start_point.z), end]
        
        waypoints = self.trajectory_gen.generate_trajectory(control_points, 0.0)
        
        return FlightPlan(
            uav_id=uav_id,
            waypoints=waypoints,
            speed=speed,
            priority=int(self.rng.integers(1, 4))
        )
    
    def generate_multi_waypoint_flight(
        self,
        uav_id: str,
        num_waypoints: int = 5,
        speed: float = 15.0
    ) -> FlightPlan:
        """
        Generate a flight with multiple waypoints.
        
        Args:
            uav_id: UAV identifier
            num_waypoints: Number of control waypoints
            speed: Flight speed in m/s
            
        Returns:
            Flight plan with multiple waypoints
        """
        # Use waypoint tour pattern
        control_points = []
        margin = 600
        
        for _ in range(num_waypoints):
            x = float(self.rng.uniform(margin, self.airspace_size[0] - margin))
            y = float(self.rng.uniform(margin, self.airspace_size[1] - margin))
            z = float(self.rng.uniform(self.min_altitude, self.max_altitude))
            control_points.append((x, y, z))
        
        waypoints = self.trajectory_gen.generate_trajectory(control_points, 0.0)
        
        return FlightPlan(
            uav_id=uav_id,
            waypoints=waypoints,
            speed=speed,
            priority=int(self.rng.integers(1, 4))
        )
    
    def generate_crossing_flights(
        self,
        uav_id1: str,
        uav_id2: str,
        speed: float = 15.0
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate two flights that intentionally cross paths.
        
        Args:
            uav_id1: First UAV identifier
            uav_id2: Second UAV identifier
            speed: Flight speed in m/s
            
        Returns:
            Tuple of two crossing flight plans
        """
        # Create crossing point near center of airspace
        center_x = self.airspace_size[0] / 2
        center_y = self.airspace_size[1] / 2
        crossing_z = (self.min_altitude + self.max_altitude) / 2
        
        # Flight 1: West to East through center
        offset = 1500
        control_points_1 = [
            (center_x - offset, center_y, crossing_z),
            (center_x + offset, center_y, crossing_z)
        ]
        
        # Flight 2: South to North through center (slightly offset in altitude)
        control_points_2 = [
            (center_x, center_y - offset, crossing_z + 20),
            (center_x, center_y + offset, crossing_z + 20)
        ]
        
        waypoints_1 = self.trajectory_gen.generate_trajectory(control_points_1, 0.0)
        # No delay - both start at same time to ensure crossing conflict
        waypoints_2 = self.trajectory_gen.generate_trajectory(control_points_2, 0.0)
        
        flight1 = FlightPlan(uav_id1, waypoints_1, speed=speed, priority=1)
        flight2 = FlightPlan(uav_id2, waypoints_2, speed=speed, priority=2)
        
        return flight1, flight2
    
    def generate_scenario(
        self,
        num_uavs: int = 10,
        flight_type: str = "mixed",
        conflict_probability: float = 0.0
    ) -> List[FlightPlan]:
        """
        Generate a complete scenario with multiple UAVs.
        
        Args:
            num_uavs: Number of UAVs in the scenario
            flight_type: Type of flights to generate:
                - "mixed": Mix of all pattern types
                - "point_to_point": Only A-to-B flights
                - "patrol": Only patrol patterns
                - "survey": Only survey patterns
                - "waypoint_tour": Only multi-waypoint tours
            conflict_probability: Not used in new implementation (legacy parameter)
            
        Returns:
            List of flight plans
        """
        flights = []
        
        # Determine pattern distribution
        if flight_type == "mixed":
            pattern_types = ['point_to_point', 'patrol', 'survey', 'waypoint_tour']
        else:
            pattern_types = [flight_type]
        
        for i in range(num_uavs):
            uav_id = f"UAV_{i:03d}"
            
            # Choose pattern type
            if len(pattern_types) > 1:
                pattern = self.rng.choice(pattern_types)
            else:
                pattern = pattern_types[0]
            
            # Generate flight
            flight = self.generate_flight(uav_id, pattern_type=pattern)
            flights.append(flight)
        
        return flights
    
    def generate_single_flight(
        self,
        uav_id: Optional[str] = None,
        flight_type: str = "waypoint_tour",
        start_time: float = 0.0,
        priority: Optional[int] = None
    ) -> FlightPlan:
        """
        Generate a single UAV flight plan.
        
        Args:
            uav_id: UAV identifier (auto-generated if None)
            flight_type: Type of flight pattern
            start_time: Start time for the flight (currently unused)
            priority: Priority level (random if None)
            
        Returns:
            Single FlightPlan
        """
        if uav_id is None:
            uav_id = f"UAV_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
        
        return self.generate_flight(uav_id, pattern_type=flight_type, priority=priority)
    
    def save_swarm_to_json(
        self,
        flights: List[FlightPlan],
        filename: str,
        metadata: Optional[Dict[str, Any]] = None
    ):
        """
        Save flight swarm to JSON file with metadata.
        
        Args:
            flights: List of flight plans
            filename: Output JSON file path
            metadata: Optional metadata to include
        """
        data = {
            'metadata': {
                'created_at': datetime.now().isoformat(),
                'num_uavs': len(flights),
                'airspace_size': self.airspace_size,
                'min_altitude': self.min_altitude,
                'max_altitude': self.max_altitude,
                'generator_version': '2.0_physics_based',
                **(metadata or {})
            },
            'flights': [flight.to_dict() for flight in flights]
        }
        
        Path(filename).parent.mkdir(parents=True, exist_ok=True)
        with open(filename, 'w') as f:
            json.dump(data, f, indent=2)
        
        avg_waypoints = sum(len(f.waypoints) for f in flights) / len(flights)
        avg_duration = sum(f.duration for f in flights) / len(flights)
        
        print(f"✓ Saved swarm of {len(flights)} UAVs to {filename}")
        print(f"  Avg waypoints/UAV: {avg_waypoints:.1f}")
        print(f"  Avg flight duration: {avg_duration:.1f}s")
    
    @staticmethod
    def load_swarm_from_json(filename: str) -> Tuple[List[FlightPlan], Dict[str, Any]]:
        """
        Load flight swarm from JSON file with metadata.
        
        Args:
            filename: Input JSON file path
            
        Returns:
            Tuple of (flights list, metadata dict)
        """
        with open(filename, 'r') as f:
            data = json.load(f)
        
        flights = []
        for flight_data in data['flights']:
            waypoints = [
                Waypoint(wp['x'], wp['y'], wp['z'], wp['time'])
                for wp in flight_data['waypoints']
            ]
            
            flight = FlightPlan(
                uav_id=flight_data['uav_id'],
                waypoints=waypoints,
                speed=flight_data['speed'],
                priority=flight_data['priority'],
                start_time=datetime.fromisoformat(flight_data['start_time'])
            )
            flights.append(flight)
        
        metadata = data.get('metadata', {})
        print(f"✓ Loaded swarm of {len(flights)} UAVs from {filename}")
        if metadata:
            print(f"  Created: {metadata.get('created_at', 'N/A')}")
            print(f"  Generator: {metadata.get('generator_version', 'legacy')}")
        
        return flights, metadata
    
    def load_and_add_flight(
        self,
        swarm_file: str,
        flight_type: str = "waypoint_tour",
        uav_id: Optional[str] = None,
        priority: Optional[int] = None
    ) -> Tuple[List[FlightPlan], FlightPlan]:
        """
        Load existing swarm and add a single new test flight.
        
        Args:
            swarm_file: Path to existing swarm JSON file
            flight_type: Type of flight to add
            uav_id: UAV identifier for new flight
            priority: Priority for new flight
            
        Returns:
            Tuple of (all flights including new one, the new flight)
        """
        # Load existing swarm
        existing_flights, metadata = self.load_swarm_from_json(swarm_file)
        
        # Update generator parameters from metadata if available
        if metadata:
            self.airspace_size = tuple(metadata.get('airspace_size', self.airspace_size))
            self.min_altitude = metadata.get('min_altitude', self.min_altitude)
            self.max_altitude = metadata.get('max_altitude', self.max_altitude)
        
        # Generate new flight
        if uav_id is None:
            existing_ids = {f.uav_id for f in existing_flights}
            counter = 1
            while f"TEST_UAV_{counter:03d}" in existing_ids:
                counter += 1
            uav_id = f"TEST_UAV_{counter:03d}"
        
        new_flight = self.generate_single_flight(
            uav_id=uav_id,
            flight_type=flight_type,
            priority=priority
        )
        
        print(f"\n✓ Generated new test flight: {uav_id}")
        print(f"  Pattern: {flight_type}")
        print(f"  Waypoints: {len(new_flight.waypoints)}")
        print(f"  Duration: {new_flight.duration:.1f}s")
        
        # Combine
        all_flights = existing_flights + [new_flight]
        
        return all_flights, new_flight
    
    def save_flights_to_csv(self, flights: List[FlightPlan], filename: str):
        """Save flight plans to CSV file."""
        import pandas as pd
        
        rows = []
        for flight in flights:
            for wp in flight.waypoints:
                rows.append({
                    'uav_id': flight.uav_id,
                    'x': wp.x,
                    'y': wp.y,
                    'z': wp.z,
                    'time': wp.time,
                    'speed': flight.speed,
                    'priority': flight.priority
                })
        
        df = pd.DataFrame(rows)
        df.to_csv(filename, index=False)
        print(f"Saved {len(flights)} flights to {filename}")
    
    @staticmethod
    def load_flights_from_csv(filename: str) -> List[FlightPlan]:
        """Load flight plans from CSV file."""
        import pandas as pd
        
        df = pd.read_csv(filename)
        flights = []
        
        for uav_id in df['uav_id'].unique():
            uav_data = df[df['uav_id'] == uav_id].sort_values('time')
            
            waypoints = [
                Waypoint(row['x'], row['y'], row['z'], row['time'])
                for _, row in uav_data.iterrows()
            ]
            
            flight = FlightPlan(
                uav_id=uav_id,
                waypoints=waypoints,
                speed=uav_data['speed'].iloc[0],
                priority=int(uav_data['priority'].iloc[0])
            )
            flights.append(flight)
        
        print(f"Loaded {len(flights)} flights from {filename}")
        return flights