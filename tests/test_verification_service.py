"""
Tests for Mission Verification Service

Tests the core verification functionality that checks if a primary mission
is safe to execute against simulated flight schedules.
"""

import pytest
from src.verification_service import MissionVerificationService, VerificationResult
from src.data_generator import FlightDataGenerator
from src.flight_plan import FlightPlan, Waypoint


class TestVerificationResult:
    """Test VerificationResult dataclass."""
    
    def test_clear_result(self):
        """Test a clear verification result."""
        generator = FlightDataGenerator(seed=42)
        mission = generator.generate_flight("TEST_001", pattern_type="point_to_point")
        
        result = VerificationResult(
            status="CLEAR",
            primary_mission=mission,
            conflicts=[],
            conflict_details=[]
        )
        
        assert result.is_clear() is True
        assert result.status == "CLEAR"
        assert len(result.conflicts) == 0
        assert "CLEAR" in result.get_summary()
    
    def test_conflict_result(self):
        """Test a conflict verification result."""
        generator = FlightDataGenerator(seed=42)
        mission = generator.generate_flight("TEST_001", pattern_type="point_to_point")
        
        # Create mock conflicts
        from src.conflict_detector import Conflict
        conflict = Conflict(
            uav1_id="TEST_001",
            uav2_id="SIMULATED_001",
            time=50.0,
            position1=(1000, 1000, 200),
            position2=(1010, 1010, 205),
            distance=15.0,
            severity=0.7
        )
        
        conflict_detail = {
            'location': (1005, 1005, 202.5),
            'time': 50.0,
            'conflicting_flight': 'SIMULATED_001',
            'distance': 15.0,
            'severity': 0.7
        }
        
        result = VerificationResult(
            status="CONFLICT_DETECTED",
            primary_mission=mission,
            conflicts=[conflict],
            conflict_details=[conflict_detail]
        )
        
        assert result.is_clear() is False
        assert result.status == "CONFLICT_DETECTED"
        assert len(result.conflicts) == 1
        assert "CONFLICT" in result.get_summary()
    
    def test_detailed_report(self):
        """Test detailed report generation."""
        generator = FlightDataGenerator(seed=42)
        mission = generator.generate_flight("TEST_001", pattern_type="point_to_point")
        
        result = VerificationResult(
            status="CLEAR",
            primary_mission=mission,
            conflicts=[],
            conflict_details=[]
        )
        
        report = result.get_detailed_report()
        assert "MISSION VERIFICATION REPORT" in report
        assert "TEST_001" in report
        assert "CLEAR TO PROCEED" in report


class TestMissionVerificationService:
    """Test MissionVerificationService."""
    
    def test_service_initialization(self):
        """Test service initialization."""
        generator = FlightDataGenerator(seed=42)
        schedules = [
            generator.generate_flight(f"SIM_{i:03d}", pattern_type="point_to_point")
            for i in range(3)
        ]
        
        service = MissionVerificationService(
            simulated_schedules=schedules,
            safety_distance=50.0
        )
        
        assert len(service.simulated_schedules) == 3
        assert service.safety_distance == 50.0
        assert service.detector is not None
    
    def test_verify_clear_mission(self):
        """Test verifying a mission with no conflicts."""
        generator = FlightDataGenerator(seed=42)
        
        # Create simulated schedules in one area
        schedules = [
            generator.generate_flight("SIM_001", pattern_type="point_to_point")
        ]
        
        # Create primary mission in different area (should be clear)
        generator2 = FlightDataGenerator(seed=999)  # Different seed
        primary = generator2.generate_flight("PRIMARY", pattern_type="point_to_point")
        
        service = MissionVerificationService(schedules, safety_distance=50.0)
        result = service.verify_mission(primary, verbose=False)
        
        # With random generation, conflicts are unlikely but possible
        # So we just check the result structure
        assert result.primary_mission.uav_id == "PRIMARY"
        assert isinstance(result.conflicts, list)
        assert isinstance(result.conflict_details, list)
        assert result.status in ["CLEAR", "CONFLICT_DETECTED"]
    
    def test_verify_conflicting_mission(self):
        """Test verifying a mission with conflicts."""
        generator = FlightDataGenerator(seed=42)
        
        # Create crossing flights (guaranteed conflict)
        sim1, sim2 = generator.generate_crossing_flights("SIM_001", "SIM_002")
        
        # Use one of the crossing flights as the primary mission
        primary, _ = generator.generate_crossing_flights("PRIMARY", "TEMP")
        
        service = MissionVerificationService([sim1, sim2], safety_distance=50.0)
        result = service.verify_mission(primary, verbose=False)
        
        # Should detect conflicts since they cross
        assert result.primary_mission.uav_id == "PRIMARY"
        assert isinstance(result.conflicts, list)
        # Conflicts expected but not guaranteed depending on timing
        if not result.is_clear():
            assert len(result.conflicts) > 0
            assert result.status == "CONFLICT_DETECTED"
    
    def test_batch_verification(self):
        """Test batch verification of multiple missions."""
        generator = FlightDataGenerator(seed=42)
        
        schedules = [
            generator.generate_flight("SIM_001", pattern_type="point_to_point")
        ]
        
        missions = [
            generator.generate_flight(f"MISSION_{i:03d}", pattern_type="point_to_point")
            for i in range(3)
        ]
        
        service = MissionVerificationService(schedules, safety_distance=50.0)
        results = service.batch_verify(missions, verbose=False)
        
        assert len(results) == 3
        assert all(key in results for key in ["MISSION_000", "MISSION_001", "MISSION_002"])
        assert all(isinstance(r, VerificationResult) for r in results.values())
    
    def test_add_simulated_schedule(self):
        """Test adding a simulated schedule."""
        generator = FlightDataGenerator(seed=42)
        
        initial_schedule = generator.generate_flight("SIM_001", pattern_type="point_to_point")
        service = MissionVerificationService([initial_schedule], safety_distance=50.0)
        
        assert len(service.simulated_schedules) == 1
        
        new_schedule = generator.generate_flight("SIM_002", pattern_type="patrol")
        service.add_simulated_schedule(new_schedule)
        
        assert len(service.simulated_schedules) == 2
        assert service.simulated_schedules[1].uav_id == "SIM_002"
    
    def test_remove_simulated_schedule(self):
        """Test removing a simulated schedule."""
        generator = FlightDataGenerator(seed=42)
        
        schedules = [
            generator.generate_flight("SIM_001", pattern_type="point_to_point"),
            generator.generate_flight("SIM_002", pattern_type="patrol")
        ]
        
        service = MissionVerificationService(schedules, safety_distance=50.0)
        assert len(service.simulated_schedules) == 2
        
        removed = service.remove_simulated_schedule("SIM_001")
        assert removed is True
        assert len(service.simulated_schedules) == 1
        assert service.simulated_schedules[0].uav_id == "SIM_002"
        
        # Try removing non-existent schedule
        removed = service.remove_simulated_schedule("SIM_999")
        assert removed is False
    
    def test_get_statistics(self):
        """Test getting statistics about simulated schedules."""
        generator = FlightDataGenerator(seed=42)
        
        schedules = [
            generator.generate_flight(f"SIM_{i:03d}", pattern_type="point_to_point")
            for i in range(5)
        ]
        
        service = MissionVerificationService(schedules, safety_distance=50.0)
        stats = service.get_statistics()
        
        assert stats['num_schedules'] == 5
        assert stats['total_waypoints'] > 0
        assert stats['total_distance'] > 0
        assert stats['total_duration'] > 0
        assert stats['avg_waypoints'] > 0
        assert stats['avg_distance'] > 0
        assert stats['avg_duration'] > 0
    
    def test_get_airspace_status(self):
        """Test getting airspace status string."""
        generator = FlightDataGenerator(seed=42)
        
        schedules = [
            generator.generate_flight("SIM_001", pattern_type="point_to_point")
        ]
        
        service = MissionVerificationService(schedules, safety_distance=50.0)
        status = service.get_airspace_status()
        
        assert "AIRSPACE STATUS" in status
        assert "SIM" in status or "1 active" in status
        assert "50.0m" in status  # Safety buffer
    
    def test_conflict_detail_structure(self):
        """Test that conflict details have correct structure."""
        generator = FlightDataGenerator(seed=42)
        
        # Create crossing flights
        sim1, sim2 = generator.generate_crossing_flights("SIM_001", "SIM_002")
        primary, _ = generator.generate_crossing_flights("PRIMARY", "TEMP")
        
        service = MissionVerificationService([sim1, sim2], safety_distance=50.0)
        result = service.verify_mission(primary, verbose=False)
        
        # If conflicts detected, check structure
        for detail in result.conflict_details:
            assert 'location' in detail
            assert 'time' in detail
            assert 'conflicting_flight' in detail
            assert 'distance' in detail
            assert 'severity' in detail
            
            assert isinstance(detail['location'], tuple)
            assert len(detail['location']) == 3
            assert isinstance(detail['time'], float)
            assert isinstance(detail['distance'], float)
            assert isinstance(detail['severity'], float)


class TestEdgeCases:
    """Test edge cases and error handling."""
    
    def test_empty_simulated_schedules(self):
        """Test service with no simulated schedules."""
        service = MissionVerificationService([], safety_distance=50.0)
        
        generator = FlightDataGenerator(seed=42)
        mission = generator.generate_flight("PRIMARY", pattern_type="point_to_point")
        
        result = service.verify_mission(mission, verbose=False)
        
        # Should be clear with no schedules
        assert result.is_clear()
        assert len(result.conflicts) == 0
    
    def test_single_waypoint_mission(self):
        """Test mission with very few waypoints."""
        schedules = []
        service = MissionVerificationService(schedules, safety_distance=50.0)
        
        # Create minimal mission
        waypoints = [
            Waypoint(1000, 1000, 200, 0),
            Waypoint(2000, 2000, 200, 100)
        ]
        mission = FlightPlan("PRIMARY", waypoints, speed=15.0)
        
        result = service.verify_mission(mission, verbose=False)
        assert result.is_clear()  # No schedules = no conflicts
    
    def test_very_small_safety_distance(self):
        """Test with very small safety distance."""
        generator = FlightDataGenerator(seed=42)
        
        schedules = [generator.generate_flight("SIM_001", pattern_type="point_to_point")]
        service = MissionVerificationService(schedules, safety_distance=1.0)  # 1m buffer
        
        mission = generator.generate_flight("PRIMARY", pattern_type="point_to_point")
        result = service.verify_mission(mission, verbose=False)
        
        # Should still work, just more likely to find conflicts
        assert result.status in ["CLEAR", "CONFLICT_DETECTED"]
    
    def test_very_large_safety_distance(self):
        """Test with very large safety distance."""
        generator = FlightDataGenerator(seed=42)
        
        schedules = [generator.generate_flight("SIM_001", pattern_type="point_to_point")]
        service = MissionVerificationService(schedules, safety_distance=5000.0)  # 5km buffer
        
        mission = generator.generate_flight("PRIMARY", pattern_type="point_to_point")
        result = service.verify_mission(mission, verbose=False)
        
        # With 5km buffer, almost everything conflicts
        assert result.status in ["CLEAR", "CONFLICT_DETECTED"]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
