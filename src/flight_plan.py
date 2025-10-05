"""
Flight plan data structures and utilities.

This module defines the core data structures for representing UAV flight plans,
waypoints, and trajectories.
"""

from dataclasses import dataclass, field
from typing import List, Tuple, Optional
import numpy as np
from datetime import datetime


@dataclass
class Waypoint:
    """
    Represents a single waypoint in 3D space with time information.
    
    Attributes:
        x: X coordinate (meters, East)
        y: Y coordinate (meters, North)
        z: Z coordinate (meters, altitude)
        time: Time in seconds from start of simulation
    """
    x: float
    y: float
    z: float
    time: float
    
    def distance_to(self, other: 'Waypoint') -> float:
        """Calculate Euclidean distance to another waypoint."""
        return np.sqrt(
            (self.x - other.x)**2 + 
            (self.y - other.y)**2 + 
            (self.z - other.z)**2
        )
    
    def to_array(self) -> np.ndarray:
        """Convert waypoint to numpy array [x, y, z, t]."""
        return np.array([self.x, self.y, self.z, self.time])
    
    def __repr__(self) -> str:
        return f"Waypoint(x={self.x:.1f}, y={self.y:.1f}, z={self.z:.1f}, t={self.time:.1f})"


@dataclass
class FlightPlan:
    """
    Represents a complete flight plan for a single UAV.
    
    Attributes:
        uav_id: Unique identifier for the UAV
        waypoints: List of waypoints defining the flight path
        speed: Average cruising speed in m/s
        start_time: Absolute start time of the flight
        priority: Flight priority (higher = more important)
    """
    uav_id: str
    waypoints: List[Waypoint]
    speed: float = 15.0  # m/s
    start_time: datetime = field(default_factory=datetime.now)
    priority: int = 1
    
    def __post_init__(self):
        """Validate flight plan after initialization."""
        if len(self.waypoints) < 2:
            raise ValueError("Flight plan must have at least 2 waypoints")
        if self.speed <= 0:
            raise ValueError("Speed must be positive")
        if self.priority < 1:
            raise ValueError("Priority must be at least 1")
    
    @property
    def duration(self) -> float:
        """Total flight duration in seconds."""
        if not self.waypoints:
            return 0.0
        return self.waypoints[-1].time - self.waypoints[0].time
    
    @property
    def total_distance(self) -> float:
        """Total flight distance in meters."""
        distance = 0.0
        for i in range(len(self.waypoints) - 1):
            distance += self.waypoints[i].distance_to(self.waypoints[i + 1])
        return distance
    
    def interpolate_position(self, t: float) -> Optional[Tuple[float, float, float]]:
        """
        Interpolate UAV position at a given time.
        
        Args:
            t: Time in seconds from start of simulation
            
        Returns:
            Tuple of (x, y, z) coordinates, or None if time is out of range
        """
        # Check if time is within flight duration
        if t < self.waypoints[0].time or t > self.waypoints[-1].time:
            return None
        
        # Find the two waypoints to interpolate between
        for i in range(len(self.waypoints) - 1):
            wp1, wp2 = self.waypoints[i], self.waypoints[i + 1]
            
            if wp1.time <= t <= wp2.time:
                # Linear interpolation
                if wp2.time == wp1.time:
                    return (wp1.x, wp1.y, wp1.z)
                
                ratio = (t - wp1.time) / (wp2.time - wp1.time)
                x = wp1.x + ratio * (wp2.x - wp1.x)
                y = wp1.y + ratio * (wp2.y - wp1.y)
                z = wp1.z + ratio * (wp2.z - wp1.z)
                
                return (x, y, z)
        
        return None
    
    def get_velocity_at_time(self, t: float) -> Optional[Tuple[float, float, float]]:
        """
        Get velocity vector at a given time.
        
        Args:
            t: Time in seconds from start of simulation
            
        Returns:
            Tuple of (vx, vy, vz) velocity components in m/s, or None if out of range
        """
        # Find the segment
        for i in range(len(self.waypoints) - 1):
            wp1, wp2 = self.waypoints[i], self.waypoints[i + 1]
            
            if wp1.time <= t <= wp2.time:
                dt = wp2.time - wp1.time
                if dt == 0:
                    return (0.0, 0.0, 0.0)
                
                vx = (wp2.x - wp1.x) / dt
                vy = (wp2.y - wp1.y) / dt
                vz = (wp2.z - wp1.z) / dt
                
                return (vx, vy, vz)
        
        return None
    
    def get_bounds(self) -> Tuple[Tuple[float, float], Tuple[float, float], Tuple[float, float]]:
        """
        Get bounding box of the flight plan.
        
        Returns:
            ((min_x, max_x), (min_y, max_y), (min_z, max_z))
        """
        xs = [wp.x for wp in self.waypoints]
        ys = [wp.y for wp in self.waypoints]
        zs = [wp.z for wp in self.waypoints]
        
        return (
            (min(xs), max(xs)),
            (min(ys), max(ys)),
            (min(zs), max(zs))
        )
    
    def to_dict(self) -> dict:
        """Convert flight plan to dictionary for serialization."""
        return {
            'uav_id': self.uav_id,
            'speed': self.speed,
            'priority': self.priority,
            'start_time': self.start_time.isoformat(),
            'waypoints': [
                {'x': wp.x, 'y': wp.y, 'z': wp.z, 'time': wp.time}
                for wp in self.waypoints
            ]
        }
    
    def clone(self) -> 'FlightPlan':
        """Create a deep copy of the flight plan."""
        return FlightPlan(
            uav_id=self.uav_id,
            waypoints=[Waypoint(wp.x, wp.y, wp.z, wp.time) for wp in self.waypoints],
            speed=self.speed,
            start_time=self.start_time,
            priority=self.priority
        )
    
    def __repr__(self) -> str:
        return f"FlightPlan(uav_id={self.uav_id}, waypoints={len(self.waypoints)}, duration={self.duration:.1f}s)"

