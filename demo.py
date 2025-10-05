"""
4D Demonstration of UAV Mission Verification Service
====================================================

This demo showcases:
1. Primary mission verification against simulated schedules
2. 4D visualization (3D space + time animation)
3. Real-time conflict detection and highlighting
4. Multiple scenarios (clear and conflict cases)

The 4D aspect is represented through animated 3D visualizations where:
- Space: X, Y, Z coordinates (3 dimensions)
- Time: Animation frames showing temporal evolution (4th dimension)
"""

import sys
import os
import numpy as np
from pathlib import Path

# Add src to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.flight_plan import FlightPlan, Waypoint
from src.data_generator import FlightDataGenerator
from src.verification_service import MissionVerificationService
from src.visualization import FlightVisualizer


def create_scenario_1_safe():
    """
    Scenario 1: SAFE MISSION
    Primary mission flies through airspace with good separation
    from simulated schedules.
    """
    print("\n" + "="*70)
    print("SCENARIO 1: SAFE MISSION (Well Separated)")
    print("="*70)
    
    # Generate simulated background traffic
    generator = FlightDataGenerator(seed=42)
    
    # Create 3 simulated flights in different regions
    simulated_schedules = []
    for i in range(3):
        flight = generator.generate_flight(
            uav_id=f"SIM-{i+1}",
            pattern_type='patrol',
            priority=2
        )
        simulated_schedules.append(flight)
    
    # Primary mission in a different area
    primary_mission = generator.generate_flight(
        uav_id="PRIMARY",
        pattern_type='point_to_point',
        priority=1
    )
    
    return primary_mission, simulated_schedules


def create_scenario_2_conflict():
    """
    Scenario 2: CONFLICT DETECTED
    Primary mission conflicts with a simulated flight in both space and time.
    """
    print("\n" + "="*70)
    print("SCENARIO 2: CONFLICT DETECTED (Space-Time Intersection)")
    print("="*70)
    
    # Create flights that will cross paths
    generator = FlightDataGenerator(seed=123)
    
    # Simulated flight: East to West, passing through center at t=100
    waypoints_sim = [
        Waypoint(x=3000, y=5000, z=200, time=0.0),
        Waypoint(x=4900, y=5000, z=200, time=95.0),
        Waypoint(x=5000, y=5000, z=200, time=100.0),  # At center
        Waypoint(x=5100, y=5000, z=200, time=105.0),
        Waypoint(x=7000, y=5000, z=200, time=200.0),
    ]
    sim_flight = FlightPlan(uav_id="SIM-CROSSING", waypoints=waypoints_sim, priority=2)
    
    # Primary mission: North to South, also at center at t=100 (SAME TIME!)
    waypoints_primary = [
        Waypoint(x=5000, y=3000, z=200, time=0.0),
        Waypoint(x=5000, y=4900, z=200, time=95.0),
        Waypoint(x=5000, y=5000, z=200, time=100.0),  # CONFLICT! Both at (5000,5000,200) at t=100
        Waypoint(x=5000, y=5100, z=200, time=105.0),
        Waypoint(x=5000, y=7000, z=200, time=200.0),
    ]
    primary_mission = FlightPlan(uav_id="PRIMARY", waypoints=waypoints_primary, priority=1)
    
    # Add some background traffic
    background = []
    for i in range(2):
        flight = generator.generate_flight(
            uav_id=f"SIM-BG-{i+1}",
            pattern_type='waypoint_tour',
            priority=2
        )
        background.append(flight)
    
    return primary_mission, [sim_flight] + background


def create_scenario_3_vertical():
    """
    Scenario 3: VERTICAL SEPARATION TEST
    Primary mission flies at different altitude - should be safe.
    """
    print("\n" + "="*70)
    print("SCENARIO 3: VERTICAL SEPARATION (3D Spatial Analysis)")
    print("="*70)
    
    # Simulated flight at 200m altitude
    waypoints_sim = [
        Waypoint(x=4000, y=4000, z=200, time=0.0),
        Waypoint(x=5000, y=5000, z=200, time=100.0),
        Waypoint(x=6000, y=6000, z=200, time=200.0),
    ]
    sim_flight = FlightPlan(uav_id="SIM-LOW", waypoints=waypoints_sim, priority=2)
    
    # Primary mission at 220m altitude (same X-Y path, only 20m vertical separation)
    waypoints_primary = [
        Waypoint(x=4000, y=4000, z=220, time=0.0),
        Waypoint(x=5000, y=5000, z=220, time=100.0),
        Waypoint(x=6000, y=6000, z=220, time=200.0),
    ]
    primary_mission = FlightPlan(uav_id="PRIMARY", waypoints=waypoints_primary, priority=1)
    
    # Add one more background flight
    generator = FlightDataGenerator(seed=789)
    background = generator.generate_flight(uav_id="SIM-BG", pattern_type='patrol', priority=2)
    
    return primary_mission, [sim_flight, background]


def create_scenario_4_multiple_conflicts():
    """
    Scenario 4: MULTIPLE CONFLICTS
    Primary mission conflicts with multiple drones at different times.
    """
    print("\n" + "="*70)
    print("SCENARIO 4: MULTIPLE CONFLICTS (Sequential Conflicts)")
    print("="*70)
    
    # Primary mission: Long path that will cross 3 different drones
    waypoints_primary = [
        Waypoint(x=2000, y=2000, z=200, time=0.0),
        Waypoint(x=3500, y=3500, z=200, time=75.0),
        Waypoint(x=4000, y=4000, z=200, time=100.0),  # Will conflict with SIM-1
        Waypoint(x=4500, y=4500, z=200, time=125.0),
        Waypoint(x=5500, y=5500, z=200, time=175.0),
        Waypoint(x=6000, y=6000, z=200, time=200.0),  # Will conflict with SIM-2
        Waypoint(x=6500, y=6500, z=200, time=225.0),
        Waypoint(x=7500, y=7500, z=200, time=275.0),
        Waypoint(x=8000, y=8000, z=200, time=300.0),  # Will conflict with SIM-3
    ]
    primary_mission = FlightPlan(uav_id="PRIMARY", waypoints=waypoints_primary, priority=1)
    
    # Simulated flight 1: Passes through (4000, 4000) around same time
    waypoints_sim1 = [
        Waypoint(x=4000, y=2000, z=200, time=50.0),
        Waypoint(x=4000, y=3500, z=200, time=87.5),
        Waypoint(x=4000, y=4000, z=200, time=100.0),  # CONFLICT 1! Same as primary at t=100
        Waypoint(x=4000, y=4500, z=200, time=112.5),
        Waypoint(x=4000, y=6000, z=200, time=150.0),
    ]
    sim1 = FlightPlan(uav_id="SIM-CROSS-1", waypoints=waypoints_sim1, priority=2)
    
    # Simulated flight 2: Passes through (6000, 6000) around same time
    waypoints_sim2 = [
        Waypoint(x=8000, y=6000, z=200, time=150.0),
        Waypoint(x=7000, y=6000, z=200, time=175.0),
        Waypoint(x=6000, y=6000, z=200, time=200.0),  # CONFLICT 2! Same as primary at t=200
        Waypoint(x=5000, y=6000, z=200, time=225.0),
        Waypoint(x=4000, y=6000, z=200, time=250.0),
    ]
    sim2 = FlightPlan(uav_id="SIM-CROSS-2", waypoints=waypoints_sim2, priority=2)
    
    # Simulated flight 3: At (8000, 8000) around same time
    waypoints_sim3 = [
        Waypoint(x=8000, y=7000, z=200, time=250.0),
        Waypoint(x=8000, y=7500, z=200, time=275.0),
        Waypoint(x=8000, y=8000, z=200, time=300.0),  # CONFLICT 3! Same as primary at t=300
        Waypoint(x=8000, y=8500, z=200, time=325.0),
        Waypoint(x=8000, y=9000, z=200, time=350.0),
    ]
    sim3 = FlightPlan(uav_id="SIM-HOVER", waypoints=waypoints_sim3, priority=2)
    
    return primary_mission, [sim1, sim2, sim3]


def run_scenario(scenario_name, primary_mission, simulated_schedules, base_output_dir):
    """Run a verification scenario and create 4D visualization."""
    
    # Create scenario folder
    scenario_folder = scenario_name.lower().replace(' ', '_')
    output_dir = os.path.join(base_output_dir, 'demo_4d', scenario_folder)
    os.makedirs(output_dir, exist_ok=True)
    
    # Initialize verification service
    service = MissionVerificationService(
        simulated_schedules=simulated_schedules,
        safety_distance=10.0,
        time_step=0.5
    )
    
    # Verify the mission
    print("\n🔍 Verifying primary mission...")
    result = service.verify_mission(primary_mission, verbose=True)
    
    # Print detailed results
    print("\n" + "-"*70)
    print("VERIFICATION RESULT")
    print("-"*70)
    
    if result.is_clear():
        print("✅ Status: CLEAR - Mission is safe to execute")
        print(f"   Primary mission: {primary_mission.uav_id}")
        print(f"   Duration: {primary_mission.duration:.1f}s")
        print(f"   Distance: {primary_mission.total_distance:.1f}m")
        print(f"   Checked against {len(simulated_schedules)} simulated flights")
    else:
        print("❌ Status: CONFLICT DETECTED - Mission is NOT safe")
        print(f"   Number of conflicts: {len(result.conflicts)}")
        print("\n   Conflict Details:")
        for i, conflict in enumerate(result.conflicts, 1):
            midpoint = (
                (conflict.position1[0] + conflict.position2[0]) / 2,
                (conflict.position1[1] + conflict.position2[1]) / 2,
                (conflict.position1[2] + conflict.position2[2]) / 2
            )
            print(f"\n   Conflict #{i}:")
            print(f"   • Time: {conflict.time:.1f}s")
            print(f"   • Location: ({midpoint[0]:.0f}, {midpoint[1]:.0f}, {midpoint[2]:.0f})")
            print(f"   • Distance: {conflict.distance:.1f}m")
            print(f"   • Flights: {conflict.uav1_id} ↔ {conflict.uav2_id}")
    
    print("-"*70)
    
    # Create visualizations
    print("\n📊 Creating 4D visualizations...")
    
    viz = FlightVisualizer(figsize=(14, 10), dpi=150)
    all_flights = [primary_mission] + simulated_schedules
    
    # 1. Static 3D view
    print("   • 3D spatial view...")
    import matplotlib.pyplot as plt
    viz.plot_flights_3d(
        all_flights,
        conflicts=result.conflicts if not result.is_clear() else [],
        title=f"{scenario_name} - 3D View",
        show_waypoints=True
    )
    plt.savefig(os.path.join(output_dir, '3d_static.png'), dpi=150, bbox_inches='tight')
    plt.close()
    
    # 2. 2D top-down view
    print("   • 2D top-down view...")
    viz.plot_flights_2d_top(
        all_flights,
        conflicts=result.conflicts if not result.is_clear() else [],
        title=f"{scenario_name} - 2D Top View"
    )
    plt.savefig(os.path.join(output_dir, '2d_topdown.png'), dpi=150, bbox_inches='tight')
    plt.close()
    
    # 3. Altitude profile
    print("   • Altitude profile...")
    viz.plot_altitude_profile(
        all_flights,
        conflicts=result.conflicts if not result.is_clear() else [],
        title=f"{scenario_name} - Altitude Profile"
    )
    plt.savefig(os.path.join(output_dir, 'altitude_profile.png'), dpi=150, bbox_inches='tight')
    plt.close()
    
    # 4. Save verification summary as JSON
    print("   • Verification summary (JSON)...")
    json_path = os.path.join(output_dir, 'verification_summary.json')
    result.save_json(json_path)
    
    # 5. 4D ANIMATION (3D space + time)
    print("   • 4D animation (3D + time)...")
    print("     This shows the temporal evolution of the scenario")
    anim = viz.animate_3d(
        all_flights,
        conflicts=result.conflicts if not result.is_clear() else [],
        fps=30,
        duration=10
    )
    viz.save_animation(anim, os.path.join(output_dir, '4d_animation.mp4'))
    print(f"     ✅ Saved as: 4d_animation.mp4")
    
    # 6. 2D animated top-down (alternative 4D view)
    print("   • 2D top-down animation...")
    anim = viz.animate_2d_topdown(
        all_flights,
        conflicts=result.conflicts if not result.is_clear() else [],
        fps=30,
        duration=10
    )
    viz.save_animation(anim, os.path.join(output_dir, '4d_topdown.mp4'))
    print(f"     ✅ Saved as: 4d_topdown.mp4")
    
    # 7. Altitude animation
    print("   • Altitude profile animation...")
    anim = viz.animate_altitude_profile(
        all_flights,
        conflicts=result.conflicts if not result.is_clear() else [],
        fps=30,
        duration=10
    )
    viz.save_animation(anim, os.path.join(output_dir, 'altitude_animation.mp4'))
    print(f"     ✅ Saved as: altitude_animation.mp4")
    
    print(f"\n✅ Visualizations complete! Saved to: {output_dir}/")
    print(f"   Files: 3d_static.png, 2d_topdown.png, altitude_profile.png, verification_summary.json")
    print(f"          + 4d_animation.mp4, 4d_topdown.mp4, altitude_animation.mp4")
    
    return result


def main():
    """Run comprehensive 4D demonstration."""
    
    print("\n╔═══════════════════════════════════════════════════════════════════╗")
    print("║                                                                   ║")
    print("║        UAV MISSION VERIFICATION - 4D DEMONSTRATION                ║")
    print("║                                                                   ║")
    print("║     Showcasing 4D Conflict Detection (3D Space + Time)            ║")
    print("║                                                                   ║")
    print("╚═══════════════════════════════════════════════════════════════════╝")
    
    # Setup output directory
    output_dir = "outputs"
    os.makedirs(output_dir, exist_ok=True)
    
    print("\nThis demonstration will:")
    print("  1. Generate primary missions and simulated flight schedules")
    print("  2. Verify mission safety using 4D space-time analysis")
    print("  3. Create animated visualizations showing temporal evolution")
    print("  4. Highlight conflicts in both space AND time")
    print("\nThe '4D' aspect:")
    print("  • 3 spatial dimensions: X, Y, Z coordinates")
    print("  • 1 temporal dimension: Time (shown through animation)")
    print("  • Conflicts are detected when drones are close in ALL 4 dimensions")
    
    # Run all scenarios
    results = []
    
    # Scenario 1: Safe mission
    primary1, simulated1 = create_scenario_1_safe()
    result1 = run_scenario("Scenario_1_Safe", primary1, simulated1, output_dir)
    results.append(("Safe Mission", result1))
    
    # Scenario 2: Conflict
    primary2, simulated2 = create_scenario_2_conflict()
    result2 = run_scenario("Scenario_2_Conflict", primary2, simulated2, output_dir)
    results.append(("Conflict Detected", result2))
    
    # Scenario 3: Vertical separation
    primary3, simulated3 = create_scenario_3_vertical()
    result3 = run_scenario("Scenario_3_Vertical", primary3, simulated3, output_dir)
    results.append(("Vertical Separation", result3))
    
    # Scenario 4: Multiple conflicts
    primary4, simulated4 = create_scenario_4_multiple_conflicts()
    result4 = run_scenario("Scenario_4_Multiple_Conflicts", primary4, simulated4, output_dir)
    results.append(("Multiple Conflicts", result4))
    
    # Final summary
    print("\n" + "="*70)
    print("FINAL SUMMARY - 4D DEMONSTRATION")
    print("="*70)
    
    for scenario_name, result in results:
        status = "✅ CLEAR" if result.is_clear() else "❌ CONFLICT"
        conflicts_str = "" if result.is_clear() else f" ({len(result.conflicts)} conflicts)"
        print(f"{scenario_name:25s} : {status}{conflicts_str}")
    
    print("\n📁 Output files saved to: outputs/demo_4d/")
    print("   • 4 scenarios, each with:")
    print("     - 3d_static.png")
    print("     - 2d_topdown.png")
    print("     - altitude_profile.png")
    print("     - verification_summary.json")
    print("     - 4d_animation.mp4")
    print("     - 4d_topdown.mp4")
    print("     - altitude_animation.mp4")
    print("\n🎬 4D Animations showcase:")
    print("   • Real-time path tracing")
    print("   • Temporal conflict highlighting")
    print("   • Drone movement synchronized with time")
    print("   • Multiple camera perspectives (3D and 2D top-down)")
    
    print("\n╔═══════════════════════════════════════════════════════════════════╗")
    print("║                                                                   ║")
    print("║              ✅ 4D DEMONSTRATION COMPLETE! ✅                     ║")
    print("║                                                                   ║")
    print("║   All scenarios verified with 4D space-time analysis              ║")
    print("║                                                                   ║")
    print("╚═══════════════════════════════════════════════════════════════════╝\n")


if __name__ == "__main__":
    main()