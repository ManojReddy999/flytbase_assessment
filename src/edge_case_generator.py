"""
Edge Case Generator for Mission Verification

Generates specific edge case scenarios to test the robustness of the
verification service, including:
- Parallel paths within safety buffer (different times)
- Crossing paths (different times)
- Near misses (just outside buffer)
- Boundary conditions
- Temporal separation scenarios
"""

from typing import List, Tuple
import numpy as np
from .flight_plan import FlightPlan, Waypoint


class EdgeCaseGenerator:
    """
    Generates edge case scenarios for testing verification service robustness.
    """
    
    def __init__(self, safety_distance: float = 10.0):
        """
        Initialize edge case generator.
        
        Args:
            safety_distance: Safety buffer distance in meters (default: 10.0m)
        """
        self.safety_distance = safety_distance
    
    def parallel_paths_different_times(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED",
        separation: float = 30.0,  # Less than safety buffer
        time_offset: float = 300.0  # 5 minutes apart
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate two parallel paths within safety buffer but at different times.
        
        This tests temporal separation - spatially close but temporally separated.
        Should NOT conflict if time_offset is large enough.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            separation: Lateral separation (< safety_distance)
            time_offset: Time offset between flights (seconds)
            
        Returns:
            Tuple of (primary_mission, simulated_schedule)
        """
        # Path 1: North-South at x=5000, y=3000->7000
        waypoints1 = [
            Waypoint(5000, 3000, 200, 0),
            Waypoint(5000, 4000, 200, 50),
            Waypoint(5000, 5000, 200, 100),
            Waypoint(5000, 6000, 200, 150),
            Waypoint(5000, 7000, 200, 200)
        ]
        
        # Path 2: Parallel, offset by separation distance
        # Same path but at x=5000+separation
        waypoints2 = [
            Waypoint(5000 + separation, 3000, 200, time_offset),
            Waypoint(5000 + separation, 4000, 200, time_offset + 50),
            Waypoint(5000 + separation, 5000, 200, time_offset + 100),
            Waypoint(5000 + separation, 6000, 200, time_offset + 150),
            Waypoint(5000 + separation, 7000, 200, time_offset + 200)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=15.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def crossing_paths_different_times(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED",
        time_offset: float = 200.0
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate crossing paths that intersect spatially but at different times.
        
        Should NOT conflict due to temporal separation.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            time_offset: Time offset (seconds)
            
        Returns:
            Tuple of flights
        """
        # Path 1: West to East
        waypoints1 = [
            Waypoint(3000, 5000, 200, 0),
            Waypoint(4000, 5000, 200, 50),
            Waypoint(5000, 5000, 200, 100),  # Crossing point at t=100
            Waypoint(6000, 5000, 200, 150),
            Waypoint(7000, 5000, 200, 200)
        ]
        
        # Path 2: South to North, arrives at crossing point much later
        waypoints2 = [
            Waypoint(5000, 3000, 200, time_offset),
            Waypoint(5000, 4000, 200, time_offset + 50),
            Waypoint(5000, 5000, 200, time_offset + 100),  # Crossing point at t=300
            Waypoint(5000, 6000, 200, time_offset + 150),
            Waypoint(5000, 7000, 200, time_offset + 200)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=15.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def near_miss_outside_buffer(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED",
        separation: float = None  # Defaults to safety_distance + 5m
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate paths that come close but stay outside safety buffer.
        
        Should NOT conflict - just outside buffer.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            separation: Minimum separation (defaults to safety + 5m)
            
        Returns:
            Tuple of flights
        """
        if separation is None:
            separation = self.safety_distance + 5.0
        
        # Path 1: Straight line
        waypoints1 = [
            Waypoint(5000, 5000, 200, 0),
            Waypoint(6000, 5000, 200, 100)
        ]
        
        # Path 2: Passes nearby but outside buffer
        waypoints2 = [
            Waypoint(5500, 5000 - separation, 200, 45),  # Closest at t=45
            Waypoint(5500, 5000 + separation, 200, 100)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=15.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def boundary_condition_exact_buffer(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED"
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate paths that maintain exactly the safety buffer distance.
        
        This tests boundary condition - should it conflict or not?
        Typically should NOT conflict if exactly at buffer distance.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            
        Returns:
            Tuple of flights
        """
        # Path 1: Stationary at center
        waypoints1 = [
            Waypoint(5000, 5000, 200, 0),
            Waypoint(5000, 5000, 200, 100)
        ]
        
        # Path 2: Circular path at exactly safety_distance radius
        # Using approximation with waypoints
        center_x, center_y = 5000, 5000
        radius = self.safety_distance
        
        waypoints2 = []
        for i, angle in enumerate(np.linspace(0, 2*np.pi, 9)):
            x = center_x + radius * np.cos(angle)
            y = center_y + radius * np.sin(angle)
            waypoints2.append(Waypoint(x, y, 200, i * 12.5))
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=0.1)  # Nearly stationary
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def same_waypoint_different_times(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED",
        time_gap: float = 600.0  # 10 minutes
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate flights passing through same waypoint but at different times.
        
        Should NOT conflict due to temporal separation.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            time_gap: Time gap between arrivals (seconds)
            
        Returns:
            Tuple of flights
        """
        shared_point = (5000, 5000, 200)
        
        # Flight 1: Arrives at shared point at t=100
        waypoints1 = [
            Waypoint(4000, 5000, 200, 0),
            Waypoint(*shared_point, 100),
            Waypoint(6000, 5000, 200, 200)
        ]
        
        # Flight 2: Arrives at shared point at t=700 (10 minutes later)
        waypoints2 = [
            Waypoint(5000, 4000, 200, time_gap - 100),
            Waypoint(*shared_point, time_gap),
            Waypoint(5000, 6000, 200, time_gap + 100)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=15.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def head_on_collision_course(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED",
        time_offset: float = 0.0
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate head-on collision course (same path, opposite directions).
        
        If time_offset=0, SHOULD conflict (actual collision).
        If time_offset is large, should NOT conflict (temporal separation).
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            time_offset: Time offset (0=collision, large=safe)
            
        Returns:
            Tuple of flights
        """
        # Path 1: West to East
        waypoints1 = [
            Waypoint(3000, 5000, 200, 0),
            Waypoint(5000, 5000, 200, 100),
            Waypoint(7000, 5000, 200, 200)
        ]
        
        # Path 2: East to West (opposite direction, same path)
        waypoints2 = [
            Waypoint(7000, 5000, 200, time_offset),
            Waypoint(5000, 5000, 200, time_offset + 100),
            Waypoint(3000, 5000, 200, time_offset + 200)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=15.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def vertical_stacking(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED",
        vertical_separation: float = 100.0
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate vertically stacked flights (same X-Y path, different altitudes).
        
        Should NOT conflict if vertical separation is sufficient.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            vertical_separation: Altitude difference (meters)
            
        Returns:
            Tuple of flights
        """
        # Both follow same horizontal path
        base_altitude = 200
        
        waypoints1 = [
            Waypoint(4000, 5000, base_altitude, 0),
            Waypoint(5000, 5000, base_altitude, 50),
            Waypoint(6000, 5000, base_altitude, 100)
        ]
        
        waypoints2 = [
            Waypoint(4000, 5000, base_altitude + vertical_separation, 0),
            Waypoint(5000, 5000, base_altitude + vertical_separation, 50),
            Waypoint(6000, 5000, base_altitude + vertical_separation, 100)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=15.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def fast_moving_brief_proximity(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED"
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate fast-moving UAVs that are only briefly in proximity.
        
        Tests if brief proximity is correctly identified.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            
        Returns:
            Tuple of flights
        """
        # Fast horizontal flight
        waypoints1 = [
            Waypoint(3000, 5000, 200, 0),
            Waypoint(7000, 5000, 200, 50)  # Fast (80 m/s)
        ]
        
        # Crossing fast flight
        waypoints2 = [
            Waypoint(5000, 3000, 200, 20),  # Starts later
            Waypoint(5000, 7000, 200, 70)   # Fast (80 m/s)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=80.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=80.0)
        
        return flight1, flight2
    
    def grazing_tangent_paths(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED"
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate paths that graze each other tangentially.
        
        Closest approach is exactly at safety buffer distance.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            
        Returns:
            Tuple of flights
        """
        # Path 1: Straight horizontal
        waypoints1 = [
            Waypoint(4000, 5000, 200, 0),
            Waypoint(6000, 5000, 200, 100)
        ]
        
        # Path 2: Arc that grazes path 1 at exactly safety_distance
        # At x=5000, y should be 5000 + safety_distance
        offset = self.safety_distance
        
        waypoints2 = [
            Waypoint(4500, 5000 + offset * 2, 200, 0),
            Waypoint(5000, 5000 + offset, 200, 50),  # Closest point
            Waypoint(5500, 5000 + offset * 2, 200, 100)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=15.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def start_end_point_conflict(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIMULATED",
        time_gap: float = 10.0  # Small gap
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate flights where one ends where another starts.
        
        Tests handling of start/end point proximity.
        
        Args:
            uav1_id: First UAV ID
            uav2_id: Second UAV ID
            time_gap: Time between end of flight1 and start of flight2
            
        Returns:
            Tuple of flights
        """
        handoff_point = (5000, 5000, 200)
        
        # Flight 1: Ends at handoff point at t=100
        waypoints1 = [
            Waypoint(4000, 5000, 200, 0),
            Waypoint(*handoff_point, 100)
        ]
        
        # Flight 2: Starts at handoff point at t=110 (10s later)
        waypoints2 = [
            Waypoint(*handoff_point, 100 + time_gap),
            Waypoint(6000, 5000, 200, 200 + time_gap)
        ]
        
        flight1 = FlightPlan(uav1_id, waypoints1, speed=15.0)
        flight2 = FlightPlan(uav2_id, waypoints2, speed=15.0)
        
        return flight1, flight2
    
    def get_all_edge_cases(self) -> List[Tuple[str, Tuple[FlightPlan, FlightPlan], bool]]:
        """
        Generate all edge cases with expected outcomes.
        
        Returns:
            List of (name, (primary, simulated), should_conflict) tuples
        """
        cases = [
            ("Parallel paths, different times", 
             self.parallel_paths_different_times(), 
             False),  # Should NOT conflict
            
            ("Crossing paths, different times", 
             self.crossing_paths_different_times(), 
             False),
            
            ("Near miss outside buffer", 
             self.near_miss_outside_buffer(), 
             False),
            
            ("Boundary condition at exact buffer", 
             self.boundary_condition_exact_buffer(), 
             False),  # Exactly at buffer - no conflict
            
            ("Same waypoint, different times", 
             self.same_waypoint_different_times(), 
             False),
            
            ("Head-on with time offset", 
             self.head_on_collision_course(time_offset=200.0), 
             False),
            
            ("Head-on simultaneous", 
             self.head_on_collision_course(time_offset=0.0), 
             True),  # SHOULD conflict
            
            ("Vertical stacking (sufficient)", 
             self.vertical_stacking(vertical_separation=100.0), 
             False),
            
            ("Vertical stacking (insufficient)", 
             self.vertical_stacking(vertical_separation=20.0), 
             True),  # SHOULD conflict
            
            ("Fast moving brief proximity", 
             self.fast_moving_brief_proximity(), 
             True),  # Likely conflict
            
            ("Grazing tangent paths", 
             self.grazing_tangent_paths(), 
             False),  # Exactly at buffer
            
            ("Start/end handoff", 
             self.start_end_point_conflict(time_gap=10.0), 
             False),  # 10s gap should be safe
            
            ("Start/end immediate", 
             self.start_end_point_conflict(time_gap=0.0), 
             True),  # SHOULD conflict
            
            ("Multiple conflicts (sequential)", 
             self.multiple_conflicts_sequential(), 
             True),  # Multiple conflicts
            
            ("Multiple drones conflict", 
             (lambda: (primary, sims) if (temp := self.multiple_drones_conflict()) and (primary := temp[0]) and (sims := temp[1]) else None)(), 
             True),  # Conflicts with multiple drones
        ]
        
        return cases
    
    def multiple_conflicts_sequential(
        self,
        uav1_id: str = "PRIMARY",
        uav2_id: str = "SIM-MULTI"
    ) -> Tuple[FlightPlan, FlightPlan]:
        """
        Generate scenario with multiple conflict points along the path.
        Primary mission crosses simulated path multiple times.
        
        Expected: CONFLICT (multiple conflicts)
        """
        # Primary: Zigzag pattern
        primary_waypoints = [
            Waypoint(x=0, y=0, z=150, time=0),
            Waypoint(x=1000, y=0, z=150, time=50),
            Waypoint(x=1000, y=1000, z=150, time=100),  # Conflict 1
            Waypoint(x=0, y=1000, z=150, time=150),
            Waypoint(x=0, y=2000, z=150, time=200),     # Conflict 2
            Waypoint(x=1000, y=2000, z=150, time=250),
        ]
        primary = FlightPlan(uav_id=uav1_id, waypoints=primary_waypoints)
        
        # Simulated: Straight line through the zigzag
        simulated_waypoints = [
            Waypoint(x=500, y=0, z=150, time=0),
            Waypoint(x=500, y=1000, z=150, time=100),   # Conflict 1
            Waypoint(x=500, y=2000, z=150, time=200),   # Conflict 2
            Waypoint(x=500, y=3000, z=150, time=300),
        ]
        simulated = FlightPlan(uav_id=uav2_id, waypoints=simulated_waypoints)
        
        return primary, simulated
    
    def multiple_drones_conflict(
        self,
        primary_id: str = "PRIMARY"
    ) -> Tuple[FlightPlan, List[FlightPlan]]:
        """
        Generate scenario where primary conflicts with multiple drones.
        
        Expected: CONFLICT (with multiple different drones)
        
        Returns:
            Tuple of (primary, list_of_simulated_flights)
        """
        # Primary: Diagonal path
        primary_waypoints = [
            Waypoint(x=1000, y=1000, z=200, time=0),
            Waypoint(x=3000, y=3000, z=200, time=150),
            Waypoint(x=5000, y=5000, z=200, time=300),
        ]
        primary = FlightPlan(uav_id=primary_id, waypoints=primary_waypoints)
        
        # Simulated 1: Crosses at early point
        sim1_waypoints = [
            Waypoint(x=2000, y=1000, z=200, time=0),
            Waypoint(x=2000, y=2000, z=200, time=75),  # Conflict with primary
            Waypoint(x=2000, y=3000, z=200, time=150),
        ]
        sim1 = FlightPlan(uav_id="SIM-1", waypoints=sim1_waypoints)
        
        # Simulated 2: Crosses at middle point
        sim2_waypoints = [
            Waypoint(x=4000, y=2000, z=200, time=100),
            Waypoint(x=3000, y=3000, z=200, time=150),  # Conflict with primary
            Waypoint(x=2000, y=4000, z=200, time=200),
        ]
        sim2 = FlightPlan(uav_id="SIM-2", waypoints=sim2_waypoints)
        
        # Simulated 3: Crosses at late point
        sim3_waypoints = [
            Waypoint(x=5000, y=4000, z=200, time=200),
            Waypoint(x=5000, y=5000, z=200, time=300),  # Conflict with primary
            Waypoint(x=5000, y=6000, z=200, time=400),
        ]
        sim3 = FlightPlan(uav_id="SIM-3", waypoints=sim3_waypoints)
        
        return primary, [sim1, sim2, sim3]
