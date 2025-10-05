"""
Tests for flight data generator.
"""

import pytest
import numpy as np
from src.data_generator import FlightDataGenerator
from src.flight_plan import FlightPlan, Waypoint


class TestFlightDataGenerator:
    """Tests for FlightDataGenerator class."""

    @staticmethod
    def _cross_track_distance(start: Waypoint, end: Waypoint, point: Waypoint) -> float:
        """Compute cross-track distance from a point to the line defined by start/end."""
        start_vec = np.array([start.x, start.y])
        end_vec = np.array([end.x, end.y])
        point_vec = np.array([point.x, point.y])
        line_vec = end_vec - start_vec
        if np.linalg.norm(line_vec) < 1e-6:
            return 0.0
        unit_vec = line_vec / np.linalg.norm(line_vec)
        projected = start_vec + np.dot(point_vec - start_vec, unit_vec) * unit_vec
        return float(np.linalg.norm(point_vec - projected))

    def test_generator_creation(self):
        """Test creating a generator."""
        generator = FlightDataGenerator(
            airspace_size=(10000, 10000, 500),
            min_altitude=50,
            max_altitude=400
        )
        
        assert generator.airspace_size == (10000, 10000, 500)
        assert generator.min_altitude == 50
        assert generator.max_altitude == 400
    
    def test_generator_with_seed(self):
        """Test generator with random seed for reproducibility."""
        gen1 = FlightDataGenerator(seed=42)
        gen2 = FlightDataGenerator(seed=42)
        
        wp1 = gen1.generate_random_waypoint(0.0)
        wp2 = gen2.generate_random_waypoint(0.0)
        
        assert wp1.x == wp2.x
        assert wp1.y == wp2.y
        assert wp1.z == wp2.z
    
    def test_generate_random_waypoint(self):
        """Test random waypoint generation."""
        generator = FlightDataGenerator()
        wp = generator.generate_random_waypoint(time=10.0)
        
        assert isinstance(wp, Waypoint)
        assert 0 <= wp.x <= 10000
        assert 0 <= wp.y <= 10000
        assert 50 <= wp.z <= 400
        assert wp.time == 10.0
    
    def test_generate_random_waypoint_with_z_range(self):
        """Test waypoint generation with altitude range."""
        generator = FlightDataGenerator()
        wp = generator.generate_random_waypoint(time=0.0, z_range=(100, 150))
        
        assert 100 <= wp.z <= 150
    
    def test_generate_straight_flight(self):
        """Test straight flight generation."""
        generator = FlightDataGenerator(seed=42)
        flight = generator.generate_straight_flight("UAV_001", speed=15.0)

        assert isinstance(flight, FlightPlan)
        assert flight.uav_id == "UAV_001"
        assert len(flight.waypoints) >= 2

        times = [wp.time for wp in flight.waypoints]
        assert times == sorted(times)

        start = flight.waypoints[0]
        end = flight.waypoints[-1]
        horiz_distance = np.hypot(end.x - start.x, end.y - start.y)
        assert horiz_distance > 100.0  # should travel a meaningful distance

        # Intermediate points should stay close to the start-end line (roughly straight)
        if len(flight.waypoints) > 2:
            for wp in flight.waypoints[1:-1]:
                cross_track = self._cross_track_distance(start, end, wp)
                assert cross_track < 250.0  # maintain a mostly straight trajectory
        assert flight.speed == 15.0
    
    def test_straight_flight_with_waypoints(self):
        """Test straight flight with specific waypoints."""
        generator = FlightDataGenerator()
        
        start = Waypoint(0, 0, 100, 0)
        end = Waypoint(1000, 0, 100, 100)
        
        flight = generator.generate_straight_flight(
            "UAV_001",
            start_point=start,
            end_point=end,
            speed=10.0
        )

        assert flight.waypoints[0].x == 0
        assert np.isclose(flight.waypoints[-1].x, 1000, atol=5.0)
        assert np.isclose(flight.waypoints[-1].y, 0, atol=5.0)
    
    def test_generate_multi_waypoint_flight(self):
        """Test multi-waypoint flight generation."""
        generator = FlightDataGenerator(seed=42)
        flight = generator.generate_multi_waypoint_flight(
            "UAV_002",
            num_waypoints=5,
            speed=15.0
        )

        assert isinstance(flight, FlightPlan)
        assert len(flight.waypoints) >= 5
        assert flight.waypoints[0].time == 0.0

        times = [wp.time for wp in flight.waypoints]
        assert times == sorted(times)

        # Ensure all waypoints lie within the defined airspace
        for wp in flight.waypoints:
            assert 0 <= wp.x <= 10000
            assert 0 <= wp.y <= 10000
            assert 50 <= wp.z <= 400
    
    def test_generate_crossing_flights(self):
        """Test crossing flights generation."""
        generator = FlightDataGenerator(seed=42)
        flight1, flight2 = generator.generate_crossing_flights(
            "UAV_001",
            "UAV_002",
            speed=15.0
        )
        
        assert isinstance(flight1, FlightPlan)
        assert isinstance(flight2, FlightPlan)
        assert flight1.uav_id == "UAV_001"
        assert flight2.uav_id == "UAV_002"
    
    def test_generate_scenario(self):
        """Test complete scenario generation."""
        generator = FlightDataGenerator(seed=42)
        flights = generator.generate_scenario(
            num_uavs=10,
            flight_type="mixed",
            conflict_probability=0.3
        )
        
        assert len(flights) == 10
        assert all(isinstance(f, FlightPlan) for f in flights)
        
        # All UAVs should have unique IDs
        uav_ids = [f.uav_id for f in flights]
        assert len(set(uav_ids)) == 10
    
    def test_generate_scenario_straight_only(self):
        """Test scenario with only straight flights."""
        generator = FlightDataGenerator(seed=42)
        flights = generator.generate_scenario(
            num_uavs=5,
            flight_type="straight"
        )

        # All flights should be mostly straight with monotonic timing
        for flight in flights:
            assert len(flight.waypoints) >= 2
            times = [wp.time for wp in flight.waypoints]
            assert times == sorted(times)

            start = flight.waypoints[0]
            end = flight.waypoints[-1]
            for wp in flight.waypoints[1:-1]:
                cross_track = self._cross_track_distance(start, end, wp)
                assert cross_track < 2500.0
    
    def test_generate_scenario_multi_waypoint_only(self):
        """Test scenario with only multi-waypoint flights."""
        generator = FlightDataGenerator(seed=42)
        flights = generator.generate_scenario(
            num_uavs=5,
            flight_type="multi_waypoint"
        )
        
        # All flights should have more than 2 waypoints
        assert all(len(f.waypoints) > 2 for f in flights)
    
    def test_save_and_load_csv(self, tmp_path):
        """Test saving and loading flights to/from CSV."""
        generator = FlightDataGenerator(seed=42)
        flights = generator.generate_scenario(num_uavs=3)
        
        # Save
        csv_file = tmp_path / "test_flights.csv"
        generator.save_flights_to_csv(flights, str(csv_file))
        
        assert csv_file.exists()
        
        # Load
        loaded_flights = FlightDataGenerator.load_flights_from_csv(str(csv_file))
        
        assert len(loaded_flights) == len(flights)
        
        # Compare
        for orig, loaded in zip(flights, loaded_flights):
            assert orig.uav_id == loaded.uav_id
            assert len(orig.waypoints) == len(loaded.waypoints)
            assert abs(orig.speed - loaded.speed) < 1e-6
