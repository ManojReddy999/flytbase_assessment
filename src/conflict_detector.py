"""
Conflict detection algorithms for UAV flight plans.

This module implements 4D (space-time) conflict detection between UAVs.
"""

from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np
from scipy.spatial import cKDTree

from .flight_plan import FlightPlan


@dataclass
class Conflict:
    """
    Represents a detected conflict between two UAVs.
    
    Attributes:
        uav1_id: ID of first UAV
        uav2_id: ID of second UAV
        time: Time of closest approach (seconds)
        position1: Position of UAV1 at conflict time (x, y, z)
        position2: Position of UAV2 at conflict time (x, y, z)
        distance: Minimum distance between UAVs (meters)
        severity: Conflict severity (0-1, higher is more severe)
    """
    uav1_id: str
    uav2_id: str
    time: float
    position1: Tuple[float, float, float]
    position2: Tuple[float, float, float]
    distance: float
    severity: float
    
    def __repr__(self) -> str:
        return (f"Conflict({self.uav1_id} vs {self.uav2_id}, "
                f"t={self.time:.1f}s, dist={self.distance:.1f}m, "
                f"severity={self.severity:.2f})")


class ConflictDetector:
    """
    Detects conflicts between UAV flight plans using 4D space-time analysis.
    """
    
    def __init__(
        self,
        safety_distance: float = 10.0,
        time_step: float = 1.0,
        time_window: float = 10.0,
        vertical_weight: float = 1.5
    ):
        """
        Initialize the conflict detector.
        
        Args:
            safety_distance: Minimum safe distance between UAVs (meters, default: 10.0m)
            time_step: Time resolution for sampling (seconds)
            time_window: Look-ahead time window for conflicts (seconds)
            vertical_weight: Weight for vertical separation (higher = more important)
        """
        self.safety_distance = safety_distance
        self.time_step = time_step
        self.time_window = time_window
        self.vertical_weight = vertical_weight
    
    def find_conflicts(self, flights: List[FlightPlan]) -> List[Conflict]:
        """
        Find all conflicts in a set of flight plans.

        Args:
            flights: List of flight plans to check

        Returns:
            List of detected conflicts
        """
        conflicts = []

        # Check each pair of flights
        for i in range(len(flights)):
            for j in range(i + 1, len(flights)):
                flight_conflicts = self._check_pair(flights[i], flights[j])
                conflicts.extend(flight_conflicts)

        # Sort by time
        conflicts.sort(key=lambda c: c.time)

        return conflicts

    def _calculate_max_segment_speed(self, flight: FlightPlan) -> float:
        """Estimate maximum segment speed for adaptive sampling."""
        max_speed = max(flight.speed, 0.0)

        for i in range(len(flight.waypoints) - 1):
            wp1, wp2 = flight.waypoints[i], flight.waypoints[i + 1]
            dt = wp2.time - wp1.time
            if dt <= 0:
                continue

            distance = wp1.distance_to(wp2)
            segment_speed = distance / dt
            if segment_speed > max_speed:
                max_speed = segment_speed

        return max_speed

    def _get_effective_time_step(
        self,
        flight1: FlightPlan,
        flight2: FlightPlan
    ) -> float:
        """Calculate adaptive time step based on relative speed."""
        base_step = max(self.time_step, 1e-3)

        speed1 = self._calculate_max_segment_speed(flight1)
        speed2 = self._calculate_max_segment_speed(flight2)
        relative_speed = speed1 + speed2

        if relative_speed <= 1e-6:
            return base_step

        # Sample at least four times while crossing the safety distance
        desired_step = self.safety_distance / (relative_speed * 4.0)

        # Never sample more coarsely than the configured step and
        # avoid excessive sampling by clamping to 10% of base step.
        min_step = base_step * 0.1
        effective_step = min(base_step, max(desired_step, min_step))

        return effective_step

    def _check_pair(
        self,
        flight1: FlightPlan,
        flight2: FlightPlan
    ) -> List[Conflict]:
        """
        Check for conflicts between two specific flights.
        
        Args:
            flight1: First flight plan
            flight2: Second flight plan
            
        Returns:
            List of conflicts between these two flights
        """
        conflicts = []
        
        # Find time overlap
        t_start = max(flight1.waypoints[0].time, flight2.waypoints[0].time)
        t_end = min(flight1.waypoints[-1].time, flight2.waypoints[-1].time)

        if t_start > t_end and (t_start - t_end) > 1e-6:
            return conflicts  # No temporal overlap

        # Sample positions along the flight paths
        if abs(t_start - t_end) <= 1e-6:
            times = np.array([t_start])
        else:
            step = self._get_effective_time_step(flight1, flight2)
            duration = max(t_end - t_start, step)
            sample_count = max(int(np.ceil(duration / step)) + 1, 2)
            times = np.linspace(t_start, t_end, sample_count)

        min_distance = float('inf')
        conflict_time = None
        pos1_at_conflict = None
        pos2_at_conflict = None
        
        for t in times:
            pos1 = flight1.interpolate_position(t)
            pos2 = flight2.interpolate_position(t)
            
            if pos1 is None or pos2 is None:
                continue
            
            # Calculate weighted distance
            dx = pos1[0] - pos2[0]
            dy = pos1[1] - pos2[1]
            dz = (pos1[2] - pos2[2]) * self.vertical_weight
            
            distance = np.sqrt(dx**2 + dy**2 + dz**2)
            
            if distance < min_distance:
                min_distance = distance
                conflict_time = t
                pos1_at_conflict = pos1
                pos2_at_conflict = pos2
        
        # Check if this is an actual conflict
        if min_distance < self.safety_distance and conflict_time is not None:
            # Calculate severity (0 = at safety distance, 1 = collision)
            severity = 1.0 - (min_distance / self.safety_distance)
            severity = max(0.0, min(1.0, severity))
            
            conflict = Conflict(
                uav1_id=flight1.uav_id,
                uav2_id=flight2.uav_id,
                time=conflict_time,
                position1=pos1_at_conflict,
                position2=pos2_at_conflict,
                distance=min_distance,
                severity=severity
            )
            conflicts.append(conflict)
        
        return conflicts
    
    def check_continuous_conflict(
        self,
        flight1: FlightPlan,
        flight2: FlightPlan,
        start_time: float,
        end_time: float
    ) -> bool:
        """
        Check if two flights have continuous conflict over a time period.
        
        Args:
            flight1: First flight plan
            flight2: Second flight plan
            start_time: Start of time window
            end_time: End of time window
            
        Returns:
            True if flights are in conflict for entire period
        """
        times = np.arange(start_time, end_time, self.time_step)
        
        for t in times:
            pos1 = flight1.interpolate_position(t)
            pos2 = flight2.interpolate_position(t)
            
            if pos1 is None or pos2 is None:
                return False
            
            distance = np.sqrt(
                (pos1[0] - pos2[0])**2 +
                (pos1[1] - pos2[1])**2 +
                ((pos1[2] - pos2[2]) * self.vertical_weight)**2
            )
            
            if distance >= self.safety_distance:
                return False
        
        return True
    
    def predict_conflicts(
        self,
        flights: List[FlightPlan],
        current_time: float
    ) -> List[Conflict]:
        """
        Predict conflicts within a time window from current time.
        
        Args:
            flights: List of flight plans
            current_time: Current simulation time
            
        Returns:
            List of predicted conflicts
        """
        # Temporarily modify flight plans to start at current time
        future_flights = []
        for flight in flights:
            if flight.waypoints[-1].time > current_time:
                future_flights.append(flight)
        
        # Find conflicts in the future time window
        return self.find_conflicts(future_flights)
    
    def get_conflict_matrix(self, flights: List[FlightPlan]) -> np.ndarray:
        """
        Create a conflict matrix showing which UAVs have conflicts.
        
        Args:
            flights: List of flight plans
            
        Returns:
            NxN boolean matrix where True indicates a conflict
        """
        n = len(flights)
        matrix = np.zeros((n, n), dtype=bool)
        
        flight_map = {f.uav_id: i for i, f in enumerate(flights)}
        
        conflicts = self.find_conflicts(flights)
        
        for conflict in conflicts:
            i = flight_map[conflict.uav1_id]
            j = flight_map[conflict.uav2_id]
            matrix[i, j] = True
            matrix[j, i] = True
        
        return matrix
    
    def get_statistics(self, conflicts: List[Conflict]) -> dict:
        """
        Calculate statistics about detected conflicts.
        
        Args:
            conflicts: List of conflicts
            
        Returns:
            Dictionary of statistics
        """
        if not conflicts:
            return {
                'total_conflicts': 0,
                'avg_distance': 0.0,
                'min_distance': 0.0,
                'avg_severity': 0.0,
                'max_severity': 0.0
            }
        
        distances = [c.distance for c in conflicts]
        severities = [c.severity for c in conflicts]
        
        return {
            'total_conflicts': len(conflicts),
            'avg_distance': np.mean(distances),
            'min_distance': np.min(distances),
            'max_distance': np.max(distances),
            'avg_severity': np.mean(severities),
            'max_severity': np.max(severities),
            'critical_conflicts': sum(1 for s in severities if s > 0.8)
        }
