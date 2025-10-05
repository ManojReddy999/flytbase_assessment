"""
Tests for flight plan data structures.
"""

import pytest
import numpy as np
from datetime import datetime

from src.flight_plan import Waypoint, FlightPlan


class TestWaypoint:
    """Tests for Waypoint class."""
    
    def test_waypoint_creation(self):
        """Test creating a waypoint."""
        wp = Waypoint(x=100.0, y=200.0, z=50.0, time=0.0)
        assert wp.x == 100.0
        assert wp.y == 200.0
        assert wp.z == 50.0
        assert wp.time == 0.0
    
    def test_distance_calculation(self):
        """Test distance calculation between waypoints."""
        wp1 = Waypoint(0, 0, 0, 0)
        wp2 = Waypoint(3, 4, 0, 0)
        
        distance = wp1.distance_to(wp2)
        assert distance == 5.0  # 3-4-5 triangle
    
    def test_distance_3d(self):
        """Test 3D distance calculation."""
        wp1 = Waypoint(0, 0, 0, 0)
        wp2 = Waypoint(1, 1, 1, 0)
        
        distance = wp1.distance_to(wp2)
        expected = np.sqrt(3)
        assert abs(distance - expected) < 1e-10
    
    def test_to_array(self):
        """Test conversion to numpy array."""
        wp = Waypoint(1.0, 2.0, 3.0, 4.0)
        arr = wp.to_array()
        
        assert isinstance(arr, np.ndarray)
        assert arr.shape == (4,)
        assert np.allclose(arr, [1.0, 2.0, 3.0, 4.0])


class TestFlightPlan:
    """Tests for FlightPlan class."""
    
    def test_flight_plan_creation(self):
        """Test creating a flight plan."""
        waypoints = [
            Waypoint(0, 0, 100, 0),
            Waypoint(1000, 1000, 100, 100)
        ]
        
        flight = FlightPlan(
            uav_id="UAV_001",
            waypoints=waypoints,
            speed=15.0
        )
        
        assert flight.uav_id == "UAV_001"
        assert len(flight.waypoints) == 2
        assert flight.speed == 15.0
    
    def test_flight_plan_validation(self):
        """Test flight plan validation."""
        # Should fail with single waypoint
        with pytest.raises(ValueError):
            FlightPlan(
                uav_id="UAV_001",
                waypoints=[Waypoint(0, 0, 100, 0)],
                speed=15.0
            )
        
        # Should fail with zero speed
        with pytest.raises(ValueError):
            FlightPlan(
                uav_id="UAV_001",
                waypoints=[Waypoint(0, 0, 100, 0), Waypoint(100, 100, 100, 10)],
                speed=0.0
            )
    
    def test_duration_calculation(self):
        """Test flight duration calculation."""
        waypoints = [
            Waypoint(0, 0, 100, 0),
            Waypoint(100, 100, 100, 50),
            Waypoint(200, 200, 100, 100)
        ]
        
        flight = FlightPlan("UAV_001", waypoints, speed=15.0)
        assert flight.duration == 100.0
    
    def test_total_distance(self):
        """Test total distance calculation."""
        waypoints = [
            Waypoint(0, 0, 0, 0),
            Waypoint(3, 4, 0, 10),  # Distance: 5
            Waypoint(3, 8, 0, 20)   # Distance: 4
        ]
        
        flight = FlightPlan("UAV_001", waypoints, speed=15.0)
        assert abs(flight.total_distance - 9.0) < 1e-10
    
    def test_interpolate_position(self):
        """Test position interpolation."""
        waypoints = [
            Waypoint(0, 0, 100, 0),
            Waypoint(100, 100, 200, 100)
        ]
        
        flight = FlightPlan("UAV_001", waypoints, speed=15.0)
        
        # At t=0, should be at start
        pos = flight.interpolate_position(0)
        assert pos == (0, 0, 100)
        
        # At t=50, should be halfway
        pos = flight.interpolate_position(50)
        assert pos == (50, 50, 150)
        
        # At t=100, should be at end
        pos = flight.interpolate_position(100)
        assert pos == (100, 100, 200)
        
        # Outside range should return None
        pos = flight.interpolate_position(-10)
        assert pos is None
        
        pos = flight.interpolate_position(200)
        assert pos is None
    
    def test_get_velocity(self):
        """Test velocity calculation."""
        waypoints = [
            Waypoint(0, 0, 100, 0),
            Waypoint(100, 0, 100, 10)
        ]
        
        flight = FlightPlan("UAV_001", waypoints, speed=15.0)
        
        velocity = flight.get_velocity_at_time(5)
        assert velocity is not None
        assert abs(velocity[0] - 10.0) < 1e-10  # 100m / 10s
        assert abs(velocity[1]) < 1e-10
        assert abs(velocity[2]) < 1e-10
    
    def test_get_bounds(self):
        """Test bounding box calculation."""
        waypoints = [
            Waypoint(0, 0, 100, 0),
            Waypoint(100, 200, 300, 10),
            Waypoint(50, 50, 150, 20)
        ]
        
        flight = FlightPlan("UAV_001", waypoints, speed=15.0)
        bounds = flight.get_bounds()
        
        assert bounds[0] == (0, 100)    # X bounds
        assert bounds[1] == (0, 200)    # Y bounds
        assert bounds[2] == (100, 300)  # Z bounds
    
    def test_clone(self):
        """Test flight plan cloning."""
        waypoints = [
            Waypoint(0, 0, 100, 0),
            Waypoint(100, 100, 200, 100)
        ]
        
        original = FlightPlan("UAV_001", waypoints, speed=15.0)
        clone = original.clone()
        
        # Should be equal but not same object
        assert clone.uav_id == original.uav_id
        assert len(clone.waypoints) == len(original.waypoints)
        assert clone is not original
        
        # Modifying clone shouldn't affect original
        clone.waypoints[0].x = 999
        assert original.waypoints[0].x == 0
    
    def test_to_dict(self):
        """Test conversion to dictionary."""
        waypoints = [
            Waypoint(0, 0, 100, 0),
            Waypoint(100, 100, 200, 100)
        ]
        
        flight = FlightPlan("UAV_001", waypoints, speed=15.0, priority=2)
        data = flight.to_dict()
        
        assert data['uav_id'] == "UAV_001"
        assert data['speed'] == 15.0
        assert data['priority'] == 2
        assert len(data['waypoints']) == 2

