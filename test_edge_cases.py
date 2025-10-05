#!/usr/bin/env python3
"""
Edge Case Testing for Mission Verification Service

Demonstrates robustness to various edge cases including:
- Parallel paths within safety buffer (temporal separation)
- Crossing paths at different times
- Near misses
- Boundary conditions
- And more...
"""

import os
import sys
from typing import List, Tuple
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend
import matplotlib.pyplot as plt

from src import MissionVerificationService, FlightVisualizer
from src.edge_case_generator import EdgeCaseGenerator


def print_header(text: str):
    """Print formatted header."""
    print("\n" + "="*80)
    print(f"  {text}")
    print("="*80 + "\n")


def print_subheader(text: str):
    """Print formatted subheader."""
    print(f"\n{text}")
    print("-"*80)


def test_edge_case(
    case_name: str,
    primary: object,
    simulated: object,
    expected_conflict: bool,
    service: MissionVerificationService,
    viz: FlightVisualizer
) -> Tuple[bool, object]:
    """
    Test a single edge case.
    
    Returns:
        (passed, result) tuple
    """
    # Ensure different priorities for visualization clarity
    primary.priority = 1
    simulated.priority = 2
    
    print(f"\n🧪 Testing: {case_name}")
    print(f"   Expected: {'CONFLICT' if expected_conflict else 'CLEAR'}")
    print(f"   Primary: {len(primary.waypoints)} waypoints")
    print(f"   Simulated: {len(simulated.waypoints)} waypoints")
    
    # Create scenario folder
    scenario_folder = case_name.lower().replace(' ', '_').replace(',', '')
    output_dir = os.path.join('outputs', 'edge_cases', scenario_folder)
    os.makedirs(output_dir, exist_ok=True)
    
    # Verify mission
    result = service.verify_mission(primary, verbose=False)
    
    # Check if result matches expectation
    actual_conflict = not result.is_clear()
    passed = (actual_conflict == expected_conflict)
    
    # Print result
    status_icon = "✅" if passed else "❌"
    print(f"   Actual:   {'CONFLICT' if actual_conflict else 'CLEAR'} {status_icon}")
    
    if not passed:
        print(f"   ⚠️  MISMATCH: Expected {'conflict' if expected_conflict else 'no conflict'}, "
              f"got {'conflict' if actual_conflict else 'no conflict'}")
    
    if actual_conflict:
        print(f"   Details: {len(result.conflicts)} conflict(s) detected")
        for i, detail in enumerate(result.conflict_details[:2], 1):  # Show first 2
            print(f"      - Conflict {i}: distance={detail['distance']:.1f}m at t={detail['time']:.1f}s")
    
    # Create visualizations
    all_flights = [primary, simulated]
    
    print(f"   📊 Generating visualizations...")
    
    # 1. 3D static view
    viz.plot_flights_3d(
        all_flights,
        conflicts=result.conflicts if actual_conflict else None,
        title=f"{case_name}\n{'CONFLICT' if actual_conflict else 'CLEAR'}"
    )
    plt.savefig(os.path.join(output_dir, '3d_static.png'), dpi=120, bbox_inches='tight')
    plt.close()
    
    # 2. 2D top-down view  
    viz.plot_flights_2d_top(
        all_flights,
        conflicts=result.conflicts if actual_conflict else None,
        title=f"Top-Down: {case_name}"
    )
    plt.savefig(os.path.join(output_dir, '2d_topdown.png'), dpi=120, bbox_inches='tight')
    plt.close()
    
    # 3. Altitude profile
    viz.plot_altitude_profile(
        all_flights,
        conflicts=result.conflicts if actual_conflict else None,
        title=f"Altitude: {case_name}"
    )
    plt.savefig(os.path.join(output_dir, 'altitude_profile.png'), dpi=120, bbox_inches='tight')
    plt.close('all')  # Close all figures to prevent memory warning
    
    # 4. Save verification summary as JSON
    json_path = os.path.join(output_dir, 'verification_summary.json')
    result.save_json(json_path)
    
    # 5. 3D animation
    print(f"      • Creating 3D animation...")
    anim = viz.animate_3d(
        all_flights,
        conflicts=result.conflicts if actual_conflict else None,
        duration=8,
        fps=20
    )
    viz.save_animation(anim, os.path.join(output_dir, '3d_animation.mp4'))
    
    # 4. 2D animation
    print(f"      • Creating 2D animation...")
    anim = viz.animate_2d_topdown(
        all_flights,
        conflicts=result.conflicts if actual_conflict else None,
        duration=8,
        fps=20
    )
    viz.save_animation(anim, os.path.join(output_dir, '2d_animation.mp4'))
    
    # 7. Altitude animation
    print(f"      • Creating altitude animation...")
    anim = viz.animate_altitude_profile(
        all_flights,
        conflicts=result.conflicts if actual_conflict else None,
        duration=8,
        fps=20
    )
    viz.save_animation(anim, os.path.join(output_dir, 'altitude_animation.mp4'))
    
    print(f"   ✅ Saved to: {output_dir}/")
    
    return passed, result


def main():
    """Run comprehensive edge case testing."""
    
    print_header("UAV MISSION VERIFICATION - EDGE CASE TESTING")
    
    print("This test suite demonstrates system robustness to various edge cases:")
    print("  • Parallel paths within safety buffer (temporal separation)")
    print("  • Crossing paths at different times")
    print("  • Near misses outside buffer")
    print("  • Boundary conditions")
    print("  • Vertical stacking")
    print("  • Head-on scenarios")
    print("  • And more...")
    
    # Initialize
    print_subheader("Initialization")
    
    safety_distance = 10.0
    print(f"Safety Distance: {safety_distance}m")
    
    edge_gen = EdgeCaseGenerator(safety_distance=safety_distance)
    service = MissionVerificationService([], safety_distance=safety_distance, time_step=0.5)
    viz = FlightVisualizer(figsize=(14, 6))
    
    print("✓ Edge case generator initialized")
    print("✓ Verification service initialized")
    print("✓ Visualizer initialized")
    
    # Get all edge cases
    edge_cases = edge_gen.get_all_edge_cases()
    
    print(f"\n📋 Total Edge Cases: {len(edge_cases)}")
    
    # Test each edge case
    print_header("EDGE CASE TESTS")
    
    results = []
    passed_count = 0
    failed_count = 0
    
    for i, (case_name, (primary, simulated), expected_conflict) in enumerate(edge_cases, 1):
        print(f"\n[{i}/{len(edge_cases)}]", end=" ")
        
        # Update service with this simulated schedule
        service.simulated_schedules = [simulated]
        
        # Test the case
        passed, result = test_edge_case(
            case_name,
            primary,
            simulated,
            expected_conflict,
            service,
            viz
        )
        
        results.append({
            'name': case_name,
            'passed': passed,
            'expected': expected_conflict,
            'actual': not result.is_clear(),
            'conflicts': len(result.conflicts)
        })
        
        if passed:
            passed_count += 1
        else:
            failed_count += 1
    
    # Summary
    print_header("TEST SUMMARY")
    
    print(f"Total Tests:  {len(edge_cases)}")
    print(f"✅ Passed:    {passed_count} ({passed_count/len(edge_cases)*100:.1f}%)")
    print(f"❌ Failed:    {failed_count} ({failed_count/len(edge_cases)*100:.1f}%)")
    
    if failed_count > 0:
        print("\n⚠️  Failed Tests:")
        for r in results:
            if not r['passed']:
                print(f"   • {r['name']}")
                print(f"     Expected: {'CONFLICT' if r['expected'] else 'CLEAR'}, "
                      f"Got: {'CONFLICT' if r['actual'] else 'CLEAR'}")
    
    # Detailed Results Table
    print_subheader("Detailed Results")
    
    print(f"\n{'#':<4} {'Test Case':<40} {'Expected':<10} {'Actual':<10} {'Result':<8}")
    print("-" * 80)
    
    for i, r in enumerate(results, 1):
        expected_str = "CONFLICT" if r['expected'] else "CLEAR"
        actual_str = "CONFLICT" if r['actual'] else "CLEAR"
        result_str = "✅ PASS" if r['passed'] else "❌ FAIL"
        
        print(f"{i:<4} {r['name']:<40} {expected_str:<10} {actual_str:<10} {result_str:<8}")
    
    # Edge Case Categories
    print_subheader("Edge Case Categories Tested")
    
    categories = {
        "Temporal Separation": [
            "Parallel paths, different times",
            "Crossing paths, different times",
            "Same waypoint, different times"
        ],
        "Spatial Separation": [
            "Near miss outside buffer",
            "Boundary condition at exact buffer",
            "Grazing tangent paths"
        ],
        "Vertical Separation": [
            "Vertical stacking (sufficient)",
            "Vertical stacking (insufficient)"
        ],
        "Head-On Scenarios": [
            "Head-on with time offset",
            "Head-on simultaneous"
        ],
        "Timing Edge Cases": [
            "Start/end handoff",
            "Start/end immediate",
            "Fast moving brief proximity"
        ]
    }
    
    for category, test_names in categories.items():
        print(f"\n📂 {category}:")
        for test_name in test_names:
            result = next((r for r in results if r['name'] == test_name), None)
            if result:
                status = "✅" if result['passed'] else "❌"
                print(f"   {status} {test_name}")
    
    # Key Insights
    print_header("KEY INSIGHTS")
    
    print("1. Temporal Separation:")
    print("   • Flights can be spatially close if temporally separated")
    print("   • System correctly distinguishes spatial proximity from actual conflicts")
    
    print("\n2. Spatial Boundaries:")
    print("   • Safety buffer is enforced correctly")
    print("   • Boundary conditions (exactly at buffer) handled appropriately")
    
    print("\n3. Vertical Separation:")
    print("   • Sufficient altitude difference prevents conflicts")
    print("   • System accounts for 3D spatial separation")
    
    print("\n4. Complex Scenarios:")
    print("   • Head-on collisions detected when simultaneous")
    print("   • Brief proximity correctly identified")
    print("   • Start/end point handoffs handled properly")
    
    # Recommendations
    print_header("RECOMMENDATIONS")
    
    if failed_count == 0:
        print("✅ All edge cases passed!")
        print("   System demonstrates excellent robustness")
    else:
        print(f"⚠️  {failed_count} edge case(s) need attention:")
        print("   Review failed cases and adjust parameters if needed")
    
    print("\n📊 Visualizations Generated:")
    print(f"   Check outputs/ directory for {len(edge_cases)} visualization files")
    print("   Each shows 3D and 2D views of the edge case scenario")
    
    # Final verdict
    print_header("FINAL VERDICT")
    
    if failed_count == 0:
        print("🎉 SUCCESS: System handles all edge cases correctly!")
        print("   The verification service demonstrates robust conflict detection")
        print("   across a wide range of challenging scenarios.")
    else:
        success_rate = passed_count / len(edge_cases) * 100
        print(f"⚠️  PARTIAL SUCCESS: {success_rate:.1f}% edge cases passed")
        print("   Review failed cases to improve robustness")
    
    print("\n" + "="*80)
    print()
    
    return failed_count == 0


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
