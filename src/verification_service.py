"""
Mission Verification Service

This module provides a verification service that acts as the final authority for
determining whether a primary drone's planned mission is safe to execute in shared
airspace by checking against simulated flight schedules of other drones.

The service does NOT perform deconfliction/resolution - it only verifies and reports
conflicts.
"""

from typing import List, Dict, Optional, Tuple
from dataclasses import dataclass, asdict
import json
from .flight_plan import FlightPlan
from .conflict_detector import ConflictDetector, Conflict


@dataclass
class VerificationResult:
    """Result of mission verification."""
    
    status: str  # "CLEAR" or "CONFLICT_DETECTED"
    primary_mission: FlightPlan
    conflicts: List[Conflict]
    conflict_details: List[Dict]
    
    def is_clear(self) -> bool:
        """Check if mission is clear to execute."""
        return self.status == "CLEAR"
    
    def get_summary(self) -> str:
        """Get human-readable summary of verification."""
        if self.is_clear():
            return f"✅ MISSION CLEAR - No conflicts detected for {self.primary_mission.uav_id}"
        else:
            return (f"❌ CONFLICT DETECTED - {len(self.conflicts)} conflict(s) detected "
                   f"for {self.primary_mission.uav_id}")
    
    def get_detailed_report(self) -> str:
        """Get detailed conflict report."""
        report = []
        report.append("=" * 70)
        report.append(f"MISSION VERIFICATION REPORT: {self.primary_mission.uav_id}")
        report.append("=" * 70)
        report.append("")
        
        if self.is_clear():
            report.append("✅ STATUS: CLEAR TO PROCEED")
            report.append("")
            report.append(f"Mission Duration: {self.primary_mission.duration:.1f}s")
            report.append(f"Number of Waypoints: {len(self.primary_mission.waypoints)}")
            report.append(f"Total Distance: {self.primary_mission.total_distance:.1f}m")
        else:
            report.append("❌ STATUS: CONFLICT DETECTED - MISSION NOT SAFE")
            report.append("")
            report.append(f"Number of Conflicts: {len(self.conflicts)}")
            report.append("")
            report.append("CONFLICT DETAILS:")
            report.append("-" * 70)
            
            for i, detail in enumerate(self.conflict_details, 1):
                report.append(f"\nConflict #{i}:")
                report.append(f"  Location: ({detail['location'][0]:.1f}, "
                            f"{detail['location'][1]:.1f}, {detail['location'][2]:.1f})")
                report.append(f"  Time: {detail['time']:.1f}s")
                report.append(f"  Conflicting Flight: {detail['conflicting_flight']}")
                report.append(f"  Minimum Distance: {detail['distance']:.1f}m")
                report.append(f"  Severity: {detail['severity']:.2f}")
        
        report.append("")
        report.append("=" * 70)
        return "\n".join(report)
    
    def to_dict(self) -> Dict:
        """
        Convert verification result to dictionary for JSON serialization.
        
        Returns:
            Dictionary representation of the verification result
        """
        return {
            "status": self.status,
            "is_clear": self.is_clear(),
            "primary_mission": {
                "uav_id": self.primary_mission.uav_id,
                "num_waypoints": len(self.primary_mission.waypoints),
                "duration_seconds": float(self.primary_mission.duration),
                "total_distance_meters": float(self.primary_mission.total_distance),
                "start_time": self.primary_mission.waypoints[0].time if self.primary_mission.waypoints else 0,
                "end_time": self.primary_mission.waypoints[-1].time if self.primary_mission.waypoints else 0,
            },
            "conflicts": self.conflict_details,
            "summary": {
                "total_conflicts": len(self.conflicts),
                "conflicting_flights": list(set(
                    detail['conflicting_flight'] 
                    for detail in self.conflict_details
                )) if self.conflict_details else []
            }
        }
    
    def save_json(self, filepath: str):
        """
        Save verification result as JSON file.
        
        Args:
            filepath: Path to save JSON file
        """
        with open(filepath, 'w') as f:
            json.dump(self.to_dict(), f, indent=2)
        return filepath


class MissionVerificationService:
    """
    Central verification service for checking if a primary mission is safe to execute.
    
    This service maintains a set of simulated flight schedules (background traffic) and
    provides a query interface for verifying whether a new primary mission would conflict
    with existing scheduled flights.
    
    Usage:
        # Initialize with simulated schedules
        service = MissionVerificationService(simulated_schedules)
        
        # Query for a primary mission
        result = service.verify_mission(primary_mission)
        
        if result.is_clear():
            print("Mission approved!")
        else:
            print(result.get_detailed_report())
    """
    
    def __init__(
        self,
        simulated_schedules: List[FlightPlan],
        safety_distance: float = 10.0,
        time_step: float = 0.5
    ):
        """
        Initialize the verification service.
        
        Args:
            simulated_schedules: List of pre-existing flight plans (background traffic)
            safety_distance: Minimum safe separation distance in meters (default: 10.0m)
            time_step: Time resolution for conflict checking in seconds
        """
        self.simulated_schedules = simulated_schedules
        self.safety_distance = safety_distance
        self.time_step = time_step
        self.detector = ConflictDetector(
            safety_distance=safety_distance,
            time_step=time_step
        )
        
        print(f"📡 Verification Service Initialized")
        print(f"   Simulated Schedules: {len(simulated_schedules)} flights")
        print(f"   Safety Distance: {safety_distance}m")
        print(f"   Time Resolution: {time_step}s")
    
    def verify_mission(
        self,
        primary_mission: FlightPlan,
        verbose: bool = True
    ) -> VerificationResult:
        """
        Verify if a primary mission is safe to execute.
        
        This is the main query interface. It checks the primary mission against
        all simulated schedules and returns a verification result.
        
        Args:
            primary_mission: The mission to verify
            verbose: If True, print verification progress
            
        Returns:
            VerificationResult containing status and conflict details
        """
        if verbose:
            print(f"\n🔍 Verifying Mission: {primary_mission.uav_id}")
            print(f"   Waypoints: {len(primary_mission.waypoints)}")
            print(f"   Duration: {primary_mission.duration:.1f}s")
            print(f"   Checking against {len(self.simulated_schedules)} simulated flights...")
        
        # Check for conflicts with each simulated schedule
        all_conflicts = []
        
        for simulated_flight in self.simulated_schedules:
            conflicts = self.detector.find_conflicts([primary_mission, simulated_flight])
            
            # Filter to only conflicts involving the primary mission
            relevant_conflicts = [
                c for c in conflicts
                if c.uav1_id == primary_mission.uav_id or c.uav2_id == primary_mission.uav_id
            ]
            
            all_conflicts.extend(relevant_conflicts)
        
        # Prepare conflict details
        conflict_details = []
        for conflict in all_conflicts:
            # Determine which flight is the conflicting one (not the primary)
            conflicting_id = (conflict.uav2_id 
                            if conflict.uav1_id == primary_mission.uav_id 
                            else conflict.uav1_id)
            
            # Use midpoint between the two positions as the conflict location
            midpoint = (
                (conflict.position1[0] + conflict.position2[0]) / 2,
                (conflict.position1[1] + conflict.position2[1]) / 2,
                (conflict.position1[2] + conflict.position2[2]) / 2
            )
            
            conflict_details.append({
                'location': midpoint,
                'time': conflict.time,
                'conflicting_flight': conflicting_id,
                'distance': conflict.distance,
                'severity': conflict.severity
            })
        
        # Determine status
        status = "CLEAR" if len(all_conflicts) == 0 else "CONFLICT_DETECTED"
        
        result = VerificationResult(
            status=status,
            primary_mission=primary_mission,
            conflicts=all_conflicts,
            conflict_details=conflict_details
        )
        
        if verbose:
            if result.is_clear():
                print(f"   ✅ CLEAR - No conflicts detected")
            else:
                print(f"   ❌ CONFLICT - {len(all_conflicts)} conflict(s) detected")
        
        return result
    
    def batch_verify(
        self,
        missions: List[FlightPlan],
        verbose: bool = True
    ) -> Dict[str, VerificationResult]:
        """
        Verify multiple missions at once.
        
        Args:
            missions: List of missions to verify
            verbose: If True, print progress
            
        Returns:
            Dictionary mapping UAV IDs to verification results
        """
        if verbose:
            print(f"\n📋 Batch Verification: {len(missions)} missions")
            print("=" * 70)
        
        results = {}
        for mission in missions:
            result = self.verify_mission(mission, verbose=verbose)
            results[mission.uav_id] = result
        
        if verbose:
            print("\n" + "=" * 70)
            print("BATCH SUMMARY:")
            clear_count = sum(1 for r in results.values() if r.is_clear())
            conflict_count = len(results) - clear_count
            print(f"  ✅ Clear: {clear_count}/{len(missions)}")
            print(f"  ❌ Conflicts: {conflict_count}/{len(missions)}")
        
        return results
    
    def add_simulated_schedule(self, flight: FlightPlan) -> None:
        """
        Add a new simulated schedule to the background traffic.
        
        Args:
            flight: Flight plan to add to simulated schedules
        """
        self.simulated_schedules.append(flight)
        print(f"➕ Added simulated schedule: {flight.uav_id}")
    
    def remove_simulated_schedule(self, uav_id: str) -> bool:
        """
        Remove a simulated schedule from background traffic.
        
        Args:
            uav_id: ID of the flight to remove
            
        Returns:
            True if removed, False if not found
        """
        for i, flight in enumerate(self.simulated_schedules):
            if flight.uav_id == uav_id:
                self.simulated_schedules.pop(i)
                print(f"➖ Removed simulated schedule: {uav_id}")
                return True
        return False
    
    def get_statistics(self) -> Dict:
        """Get statistics about the simulated schedules."""
        total_waypoints = sum(len(f.waypoints) for f in self.simulated_schedules)
        total_distance = sum(f.total_distance for f in self.simulated_schedules)
        total_duration = sum(f.duration for f in self.simulated_schedules)
        
        return {
            'num_schedules': len(self.simulated_schedules),
            'total_waypoints': total_waypoints,
            'total_distance': total_distance,
            'total_duration': total_duration,
            'avg_waypoints': total_waypoints / len(self.simulated_schedules) if self.simulated_schedules else 0,
            'avg_distance': total_distance / len(self.simulated_schedules) if self.simulated_schedules else 0,
            'avg_duration': total_duration / len(self.simulated_schedules) if self.simulated_schedules else 0
        }
    
    def get_airspace_status(self) -> str:
        """Get a summary of current airspace status."""
        stats = self.get_statistics()
        
        status = []
        status.append("=" * 70)
        status.append("AIRSPACE STATUS")
        status.append("=" * 70)
        status.append(f"Simulated Schedules: {stats['num_schedules']} active flights")
        status.append(f"Total Waypoints: {stats['total_waypoints']:.0f}")
        status.append(f"Total Flight Distance: {stats['total_distance']/1000:.1f} km")
        status.append(f"Average Flight Duration: {stats['avg_duration']:.1f}s")
        status.append(f"Safety Buffer: {self.safety_distance}m")
        status.append("=" * 70)
        
        return "\n".join(status)


def create_verification_service_from_file(
    filepath: str,
    safety_distance: float = 10.0,
    time_step: float = 0.5
) -> MissionVerificationService:
    """
    Create a verification service by loading simulated schedules from a file.
    
    Args:
        filepath: Path to file containing flight schedules
        safety_distance: Minimum safe separation distance
        time_step: Time resolution for checking
        
    Returns:
        Configured MissionVerificationService
    """
    from .data_generator import FlightDataGenerator
    
    generator = FlightDataGenerator()
    schedules = generator.load_from_csv(filepath)
    
    return MissionVerificationService(schedules, safety_distance, time_step)
