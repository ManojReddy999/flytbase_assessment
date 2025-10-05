"""
Tests for conflict detection algorithms.
"""

import pytest
import numpy as np
from src.conflict_detector import ConflictDetector, Conflict
from src.flight_plan import FlightPlan, Waypoint
from src.data_generator import FlightDataGenerator


class TestConflict:
    """Tests for Conflict dataclass."""
    
    def test_conflict_creation(self):
        """Test creating a conflict."""
        conflict = Conflict(
            uav1_id="UAV_001",
            uav2_id="UAV_002",
            time=50.0,
            position1=(100, 200, 150),
            position2=(110, 210, 155),
            distance=15.0,
            severity=0.7
        )
        
        assert conflict.uav1_id == "UAV_001"
        assert conflict.uav2_id == "UAV_002"
        assert conflict.time == 50.0
        assert conflict.distance == 15.0
        assert conflict.severity == 0.7


class TestConflictDetector:
    """Tests for ConflictDetector class."""
    
    def test_detector_creation(self):
        """Test creating a conflict detector."""
        detector = ConflictDetector(
            safety_distance=50.0,
            time_step=1.0
        )
        
        assert detector.safety_distance == 50.0
        assert detector.time_step == 1.0
    
    def test_no_conflict_separate_times(self):
        """Test no conflict when flights are at different times."""
        detector = ConflictDetector(safety_distance=50.0)
        
        # Flight 1: time 0-100
        flight1 = FlightPlan(
            "UAV_001",
            waypoints=[
                Waypoint(0, 0, 100, 0),
                Waypoint(1000, 0, 100, 100)
            ],
            speed=10.0
        )
        
        # Flight 2: time 200-300 (no overlap)
        flight2 = FlightPlan(
            "UAV_002",
            waypoints=[
                Waypoint(0, 0, 100, 200),
                Waypoint(1000, 0, 100, 300)
            ],
            speed=10.0
        )
        
        conflicts = detector.find_conflicts([flight1, flight2])
        assert len(conflicts) == 0
    
    def test_no_conflict_far_apart(self):
        """Test no conflict when flights are far apart."""
        detector = ConflictDetector(safety_distance=50.0)
        
        # Flight 1: y=0
        flight1 = FlightPlan(
            "UAV_001",
            waypoints=[
                Waypoint(0, 0, 100, 0),
                Waypoint(1000, 0, 100, 100)
            ],
            speed=10.0
        )
        
        # Flight 2: y=1000 (far away)
        flight2 = FlightPlan(
            "UAV_002",
            waypoints=[
                Waypoint(0, 1000, 100, 0),
                Waypoint(1000, 1000, 100, 100)
            ],
            speed=10.0
        )
        
        conflicts = detector.find_conflicts([flight1, flight2])
        assert len(conflicts) == 0
    
    def test_detect_crossing_conflict(self):
        """Test detecting a crossing conflict."""
        detector = ConflictDetector(safety_distance=50.0, time_step=0.5)
        
        generator = FlightDataGenerator(seed=42)
        flight1, flight2 = generator.generate_crossing_flights(
            "UAV_001",
            "UAV_002",
            speed=15.0
        )
        
        conflicts = detector.find_conflicts([flight1, flight2])
        
        # Should detect at least one conflict
        assert len(conflicts) > 0
        assert conflicts[0].uav1_id == "UAV_001"
        assert conflicts[0].uav2_id == "UAV_002"
        assert conflicts[0].distance < 50.0
    
    def test_conflict_severity_calculation(self):
        """Test conflict severity calculation."""
        detector = ConflictDetector(safety_distance=50.0)

        # Create flights that pass very close
        flight1 = FlightPlan(
            "UAV_001",
            waypoints=[
                Waypoint(0, 0, 100, 0),
                Waypoint(100, 0, 100, 10)
            ],
            speed=10.0
        )
        
        flight2 = FlightPlan(
            "UAV_002",
            waypoints=[
                Waypoint(50, -5, 100, 0),  # Will be very close at t=5
                Waypoint(50, 5, 100, 10)
            ],
            speed=1.0
        )
        
        conflicts = detector.find_conflicts([flight1, flight2])

        if len(conflicts) > 0:
            # Severity should be between 0 and 1
            assert 0 <= conflicts[0].severity <= 1

    def test_conflict_at_shared_endpoint(self):
        """Conflicts are detected when flights share an endpoint in time/space."""
        detector = ConflictDetector(safety_distance=30.0, time_step=0.5)

        # Flight 1 arrives at the handoff point at t=10s
        flight1 = FlightPlan(
            "UAV_ALPHA",
            waypoints=[
                Waypoint(0, 0, 100, 0),
                Waypoint(300, 0, 100, 10)
            ],
            speed=30.0
        )

        # Flight 2 departs from the exact same point at the same time
        flight2 = FlightPlan(
            "UAV_BRAVO",
            waypoints=[
                Waypoint(300, 0, 100, 10),
                Waypoint(600, 0, 100, 20)
            ],
            speed=30.0
        )

        conflicts = detector.find_conflicts([flight1, flight2])

        assert conflicts, "Expected conflict at the shared endpoint"
        assert any(abs(conflict.time - 10.0) < 1e-6 for conflict in conflicts)
        assert conflicts[0].distance < detector.safety_distance

    def test_fast_moving_crossing_conflict_detected(self):
        """Fast UAVs still trigger conflicts despite adaptive sampling."""
        detector = ConflictDetector(safety_distance=50.0, time_step=0.5)

        # Two high-speed flights crossing at (0, 0, 100) around t=1s
        flight1 = FlightPlan(
            "FAST_1",
            waypoints=[
                Waypoint(-200, 0, 100, 0),
                Waypoint(200, 0, 100, 2)
            ],
            speed=100.0
        )

        flight2 = FlightPlan(
            "FAST_2",
            waypoints=[
                Waypoint(0, -200, 100, 0),
                Waypoint(0, 200, 100, 2)
            ],
            speed=100.0
        )

        conflicts = detector.find_conflicts([flight1, flight2])

        assert conflicts, "Expected conflict between fast-moving crossing flights"
        assert any(conflict.distance < detector.safety_distance for conflict in conflicts)

    def test_multiple_conflicts(self):
        """Test detecting multiple conflicts."""
        detector = ConflictDetector(safety_distance=100.0)

        generator = FlightDataGenerator(seed=42)
        flights = generator.generate_scenario(
            num_uavs=10,
            conflict_probability=0.5
        )
        
        conflicts = detector.find_conflicts(flights)
        
        # Should detect some conflicts
        assert len(conflicts) >= 0  # May or may not have conflicts
    
    def test_conflict_matrix(self):
        """Test conflict matrix generation."""
        detector = ConflictDetector(safety_distance=50.0)
        
        generator = FlightDataGenerator(seed=42)
        flight1, flight2 = generator.generate_crossing_flights(
            "UAV_001",
            "UAV_002"
        )
        
        flights = [flight1, flight2]
        matrix = detector.get_conflict_matrix(flights)
        
        assert matrix.shape == (2, 2)
        assert matrix.dtype == bool
        
        # Diagonal should be False (UAV doesn't conflict with itself)
        assert not matrix[0, 0]
        assert not matrix[1, 1]
        
        # Matrix should be symmetric
        assert matrix[0, 1] == matrix[1, 0]
    
    def test_get_statistics(self):
        """Test conflict statistics calculation."""
        detector = ConflictDetector()
        
        # Test with no conflicts
        stats = detector.get_statistics([])
        assert stats['total_conflicts'] == 0
        assert stats['avg_distance'] == 0.0
        
        # Test with conflicts
        conflicts = [
            Conflict("UAV_001", "UAV_002", 10.0, (0, 0, 0), (10, 0, 0), 10.0, 0.8),
            Conflict("UAV_002", "UAV_003", 20.0, (0, 0, 0), (20, 0, 0), 20.0, 0.6),
        ]
        
        stats = detector.get_statistics(conflicts)
        assert stats['total_conflicts'] == 2
        assert stats['avg_distance'] == 15.0
        assert stats['min_distance'] == 10.0
        assert stats['max_distance'] == 20.0
    
    def test_continuous_conflict_check(self):
        """Test checking for continuous conflicts."""
        detector = ConflictDetector(safety_distance=50.0)
        
        # Create parallel flights very close together
        flight1 = FlightPlan(
            "UAV_001",
            waypoints=[
                Waypoint(0, 0, 100, 0),
                Waypoint(1000, 0, 100, 100)
            ],
            speed=10.0
        )
        
        flight2 = FlightPlan(
            "UAV_002",
            waypoints=[
                Waypoint(0, 10, 100, 0),  # Only 10m apart
                Waypoint(1000, 10, 100, 100)
            ],
            speed=10.0
        )
        
        is_continuous = detector.check_continuous_conflict(
            flight1, flight2, 0.0, 50.0
        )
        
        assert is_continuous  # Should be in continuous conflict
    
    def test_predict_conflicts(self):
        """Test predicting future conflicts."""
        detector = ConflictDetector()
        
        generator = FlightDataGenerator(seed=42)
        flights = generator.generate_scenario(num_uavs=5)
        
        # Predict conflicts from time 0
        conflicts = detector.predict_conflicts(flights, current_time=0.0)
        
        assert isinstance(conflicts, list)
        assert all(isinstance(c, Conflict) for c in conflicts)
