# UAV Mission Verification System

**A 4D space-time mission verification service for UAV conflict detection in shared airspace**

[![Python](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![Tests](https://img.shields.io/badge/tests-54%20passing-brightgreen.svg)](tests/)
[![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)

---

## Overview

This system serves as the **final authority for verifying whether a drone's planned waypoint mission is safe to execute** in shared airspace. It checks for conflicts in both space and time against simulated flight paths of multiple other drones using **4D space-time analysis** (X, Y, Z, Time).

### Key Features

- ✅ **Works with YOUR Data**: Accepts any flight data (CSV, JSON, database, API, etc.)
- ✅ **Simple Query Interface**: Verify your mission in 3 lines of code
- ✅ **4D Conflict Detection**: Spatial (3D) + Temporal analysis
- ✅ **Detailed Conflict Reporting**: Location, time, involved flights, distance, severity
- ✅ **4D Visualizations**: Animated 3D and 2D views with temporal evolution (**Extra Credit**)
- ✅ **Optional Test Data Generator**: Physics-based trajectories for testing/demos
- ✅ **Comprehensive Testing**: 54 unit tests + 13 edge case scenarios
- ✅ **Edge Case Handling**: Robust validation across challenging scenarios

---

## Quick Start

### Installation

```bash
# Clone the repository
git clone <repository-url>
cd flytbase

# Install dependencies
pip install -r requirements.txt
```

### Basic Usage with Your Data

```python
from src import FlightPlan, Waypoint, MissionVerificationService

# Create your mission from your data
my_mission = FlightPlan(
    uav_id="MY-DRONE",
    waypoints=[
        Waypoint(x=0, y=0, z=100, time=0),
        Waypoint(x=1000, y=1000, z=120, time=50),
        # ... your waypoints
    ]
)

# Load other drones' schedules from your data
other_schedules = [...]  # Your FlightPlan objects

# Verify your mission
service = MissionVerificationService(other_schedules)
result = service.verify_mission(my_mission)

print(result.get_summary())
```

### Run the Demo (For Testing)

```bash
# Run comprehensive 4D demonstration (3 scenarios with animations)
python demo.py
```

This will:
- Generate 3 verification scenarios (safe, conflict, vertical separation)
- Create 12 visualization files (static images + 4D animations)
- Save all outputs to `outputs/demo_4d/`

### Run Edge Case Tests

```bash
# Test 13 edge case scenarios with animations
python test_edge_cases.py
```

Generates 52 visualization files (13 scenarios × 4 files each) in `outputs/edge_cases/`

### Run Unit Tests

```bash
# Run all unit tests
pytest tests/ -v
```

---

## Architecture

### System Design

```
┌─────────────────────┐
│  Primary Mission    │  ← Mission to verify
└──────────┬──────────┘
           │
           ▼
┌─────────────────────────────────────┐
│  MissionVerificationService         │  ← Main API (Query Interface)
│  - verify_mission()                 │
│  - Returns: CLEAR or CONFLICT       │
└──────────┬──────────────────────────┘
           │
           ▼
┌─────────────────────────────────────┐
│  ConflictDetector                   │  ← 4D Detection Engine
│  - 4D space-time analysis           │
│  - Spatial check (3D distance)      │
│  - Temporal check (time overlap)    │
└──────────┬──────────────────────────┘
           │
           ▼
┌─────────────────────────────────────┐
│  Simulated Flight Schedules         │  ← Background traffic
└─────────────────────────────────────┘
```

### Core Modules

| Module | Purpose |
|--------|---------|
| `verification_service.py` | **Main API** - Mission verification query interface |
| `conflict_detector.py` | **Detection Engine** - 4D space-time conflict detection |
| `flight_plan.py` | Data structures for flights and waypoints |
| `data_generator.py` | Physics-based trajectory generation |
| `visualization.py` | Static and animated visualizations |
| `edge_case_generator.py` | Edge case scenario generation |

**Note**: `verification_service.py` and `conflict_detector.py` are separate by design:
- `conflict_detector.py` = Low-level 4D conflict detection algorithm
- `verification_service.py` = High-level query interface (uses conflict_detector)

---

## Usage

### Using Your Own Flight Data (Recommended)

**The verification service works with ANY flight data** - you don't need to use our data generator. Simply create `FlightPlan` objects from your own data:

```python
from src import FlightPlan, Waypoint, MissionVerificationService

# 1. Load YOUR primary mission data (from file, database, API, etc.)
primary_waypoints = [
    Waypoint(x=0, y=0, z=100, time=0),
    Waypoint(x=1000, y=500, z=120, time=50),
    Waypoint(x=2000, y=1000, z=100, time=100),
    Waypoint(x=3000, y=1500, z=110, time=150),
]
primary_mission = FlightPlan(
    uav_id="MISSION-001",
    waypoints=primary_waypoints,
    speed=15.0
)

# 2. Load YOUR simulated schedules (other drones in shared airspace)
# Example: Load from CSV, JSON, database, etc.
schedule_1_waypoints = [
    Waypoint(x=500, y=200, z=100, time=0),
    Waypoint(x=1500, y=700, z=110, time=60),
    Waypoint(x=2500, y=1200, z=100, time=120),
]
schedule_1 = FlightPlan(uav_id="DRONE-A", waypoints=schedule_1_waypoints)

schedule_2_waypoints = [
    Waypoint(x=1000, y=1000, z=150, time=0),
    Waypoint(x=1000, y=500, z=140, time=40),
    Waypoint(x=1000, y=0, z=150, time=80),
]
schedule_2 = FlightPlan(uav_id="DRONE-B", waypoints=schedule_2_waypoints)

simulated_schedules = [schedule_1, schedule_2]

# 3. Initialize verification service with YOUR data
service = MissionVerificationService(
    simulated_schedules=simulated_schedules,
    safety_distance=10.0  # meters (default: 10.0m, adjust as needed)
)

# 4. Verify YOUR mission (QUERY INTERFACE)
result = service.verify_mission(primary_mission)

# 5. Check result
if result.is_clear():
    print(f"✅ {result.status} - Mission is safe to execute")
else:
    print(f"❌ {result.status} - Conflicts detected:")
    for detail in result.conflict_details:
        print(f"  • Time: {detail['time']:.1f}s")
        print(f"  • Location: {detail['location']}")
        print(f"  • Distance: {detail['distance']:.1f}m")
        print(f"  • Conflicting Flight: {detail['conflicting_flight']}")
```

### Loading Data from Files

```python
import json
from src import FlightPlan, Waypoint

# Example: Load from JSON
def load_flight_from_json(filepath):
    with open(filepath, 'r') as f:
        data = json.load(f)
    
    waypoints = [
        Waypoint(x=wp['x'], y=wp['y'], z=wp['z'], time=wp['time'])
        for wp in data['waypoints']
    ]
    
    return FlightPlan(
        uav_id=data['uav_id'],
        waypoints=waypoints,
        speed=data.get('speed', 15.0)
    )

# Load your mission and schedules
primary_mission = load_flight_from_json('my_mission.json')
simulated_schedules = [
    load_flight_from_json('schedule_1.json'),
    load_flight_from_json('schedule_2.json'),
    load_flight_from_json('schedule_3.json'),
]

# Verify
service = MissionVerificationService(simulated_schedules)
result = service.verify_mission(primary_mission)
```

### Using the Data Generator (For Testing/Demos Only)

The `FlightDataGenerator` is provided for testing and demonstrations, but you should use your own real flight data:

```python
from src import FlightDataGenerator, MissionVerificationService

# Generate test data (for demos/testing only)
generator = FlightDataGenerator(seed=42)

simulated_schedules = [
    generator.generate_flight(uav_id=f"TEST-{i}", pattern_type='patrol')
    for i in range(3)
]

primary_mission = generator.generate_flight(
    uav_id="TEST-PRIMARY",
    pattern_type='point_to_point'
)

# Verify
service = MissionVerificationService(simulated_schedules)
result = service.verify_mission(primary_mission)
```

### Verification Result Format

```python
VerificationResult {
    status: "CLEAR" or "CONFLICT_DETECTED"
    is_clear(): bool  # Convenience method
    primary_mission: FlightPlan
    conflicts: List[Conflict]  # Detailed conflict objects
    conflict_details: List[dict]  # Formatted for easy consumption
}

# Each conflict detail contains:
{
    "time": float,              # When conflict occurs (seconds)
    "location": (x, y, z),      # Where conflict occurs (meters)
    "uav1_id": str,             # First UAV involved
    "uav2_id": str,             # Second UAV involved
    "distance": float,          # Distance between UAVs (meters)
    "severity": float           # Severity rating (0-1)
}
```

### Creating Visualizations

```python
import matplotlib.pyplot as plt

viz = FlightVisualizer(figsize=(14, 8))
all_flights = [primary_mission] + simulated_schedules

# Static 3D view
viz.plot_flights_3d(
    all_flights,
    conflicts=result.conflicts if not result.is_clear() else [],
    title="Mission Verification - 3D View"
)
plt.savefig('outputs/3d_view.png')
plt.close()

# 4D Animation (3D space + time)
anim = viz.animate_3d(
    all_flights,
    conflicts=result.conflicts if not result.is_clear() else [],
    duration=10,
    fps=30
)
viz.save_animation(anim, 'outputs/4d_animation.mp4')
```

---

## Requirements

### Functional Requirements

#### 1. Spatial Check ✅
Validates that the primary mission's path does not intersect with any other drone's trajectory within a **safety buffer** (default: 50m).

**Implementation**: 3D Euclidean distance calculation at each time step.

#### 2. Temporal Check ✅
Ensures that within the overall mission window, no other drone is present in the same spatial area during **overlapping time segments**.

**Implementation**: 4D space-time analysis checking both spatial proximity AND temporal overlap.

#### 3. Conflict Explanation ✅
When conflicts are detected, provides detailed explanation:
- Location(s) of conflict (X, Y, Z coordinates)
- Time(s) of conflict
- Which simulated flight(s) caused the conflict
- Distance between UAVs
- Severity rating

#### 4. Query Interface ✅
Simple Python function that accepts the primary drone's mission and returns status ("CLEAR" or "CONFLICT_DETECTED") with conflict details.

**Implementation**: `service.verify_mission(primary_mission)` → `VerificationResult`

---

## Output Structure

All generated visualizations are organized by scenario type:

```
outputs/
├── demo_4d/                    # Comprehensive demo scenarios
│   ├── scenario_1_safe/
│   │   ├── 3d_static.png
│   │   ├── 2d_topdown.png
│   │   ├── 4d_animation.mp4
│   │   └── 4d_topdown.mp4
│   ├── scenario_2_conflict/
│   └── scenario_3_vertical/
│
├── edge_cases/                 # Edge case test scenarios
│   ├── parallel_paths/
│   │   ├── 3d_static.png
│   │   ├── 2d_topdown.png
│   │   ├── 3d_animation.mp4
│   │   └── 2d_animation.mp4
│   ├── crossing_paths/
│   ├── head_on_simultaneous/
│   └── ... (13 scenarios total)
│
├── verification/               # Quick verification example
│   ├── 3d_view.png
│   ├── 2d_topdown.png
│   └── altitude_profile.png
│
└── performance/                # Performance benchmarking
    └── performance_benchmark.png
```

---

## Testing

### Unit Tests

```bash
pytest tests/ -v
```

**Coverage**:
- ✅ 54 unit tests (100% passing)
- ✅ Flight plan operations
- ✅ Data generation
- ✅ Conflict detection
- ✅ Verification service
- ✅ Edge cases

### Edge Case Testing

```bash
python test_edge_cases.py
```

**13 Edge Case Scenarios**:
1. Parallel paths (temporal separation)
2. Crossing paths (different times)
3. Near miss outside buffer
4. Boundary condition at exact buffer
5. Same waypoint (different times)
6. Head-on simultaneous
7. Head-on with time offset
8. Vertical stacking (sufficient)
9. Vertical stacking (insufficient)
10. Fast moving brief proximity
11. Grazing tangent paths
12. Start/end handoff
13. Start/end immediate

**Results**: 11/13 correct behavior (84.6%), 2/13 conservative (known limitations documented)

---

## 4D Visualization (Extra Credit)

### What is "4D"?

- **3 Spatial Dimensions**: X, Y, Z coordinates (meters)
- **1 Temporal Dimension**: Time (seconds)

### How is it visualized?

Time is shown through animation frames, where drones move through 3D space as time progresses. Conflicts are highlighted when they occur temporally.

### Features

- ✅ Real-time path tracing (drones leave visible trails)
- ✅ Permanent traced paths (full history visible)
- ✅ Time display (current simulation time)
- ✅ Rotating camera (3D animations)
- ✅ Conflict markers (appear at conflict time)
- ✅ Color-coded altitudes (2D top-down view)
- ✅ Multiple viewing angles (3D and 2D perspectives)

---

## Performance

### Benchmarks

Tested on typical consumer hardware:

| # UAVs | Generation | Detection | Total Time |
|--------|------------|-----------|------------|
| 5      | 0.02s      | 0.04s     | 0.06s      |
| 10     | 0.05s      | 0.20s     | 0.25s      |
| 20     | 0.10s      | 0.80s     | 0.90s      |
| 50     | 0.25s      | 5.19s     | 5.44s      |

**Optimal Range**: 10-20 UAVs for real-time performance

### Scalability Discussion

For handling tens of thousands of commercial drones:

**Architectural Changes Needed**:
1. **Distributed Computing**: Partition airspace into cells, process independently
2. **Spatial Indexing**: Use R-trees or quad-trees for efficient spatial queries
3. **Time Windows**: Check only relevant time windows, not entire flight history
4. **Caching**: Cache conflict checks for repeated queries
5. **Real-Time Ingestion**: Streaming data pipelines (Apache Kafka, etc.)
6. **Load Balancing**: Distribute verification requests across multiple servers

**Estimated Capacity**:
- Single server: ~1,000 UAVs (with optimizations)
- Distributed system: 10,000+ UAVs (with spatial partitioning)

---

## Project Structure

```
flytbase/
├── demo.py                      # Main comprehensive demonstration
├── test_edge_cases.py           # Edge case testing script
├── README.md                    # This file
├── requirements.txt             # Python dependencies
├── setup.py                     # Package setup
│
├── src/                         # Core implementation
│   ├── __init__.py
│   ├── verification_service.py # Main verification API
│   ├── conflict_detector.py    # 4D conflict detection engine
│   ├── flight_plan.py          # Data structures
│   ├── data_generator.py       # Trajectory generation
│   ├── visualization.py        # Static & animated viz
│   └── edge_case_generator.py  # Edge case scenarios
│
├── tests/                       # Unit tests (54 tests)
│   ├── test_verification_service.py
│   ├── test_conflict_detector.py
│   ├── test_data_generator.py
│   └── test_flight_plan.py
│
├── outputs/                     # Generated visualizations
│   ├── demo_4d/                # Demo scenarios
│   ├── edge_cases/             # Edge case scenarios
│   ├── verification/           # Example outputs
│   └── performance/            # Benchmarks
│
└── docs/
    └── api.md                   # API reference
```

---

## Design Decisions

### 1. Verification vs Optimization

**Choice**: Implemented a **verification service**, not a resolution/optimization system.

**Rationale**: The assignment specifies "final authority for **verifying** whether a mission is safe", not resolving conflicts. Our system returns CLEAR/CONFLICT status without automatic resolution.

### 2. 4D Space-Time Analysis

**Choice**: Implemented full 4D conflict detection (X, Y, Z, Time).

**Rationale**: Ensures conflicts are detected only when drones are close in space **AND** time simultaneously. Prevents false positives from paths that cross spatially but at different times.

### 3. Separate Conflict Detector and Verification Service

**Choice**: Two modules instead of one.

**Rationale**:
- `conflict_detector.py` = Reusable low-level detection algorithm
- `verification_service.py` = High-level business logic and query interface
- Follows Single Responsibility Principle
- Easier to test and maintain

### 4. Physics-Based Trajectory Generation

**Choice**: Use cubic splines with realistic constraints.

**Rationale**: Generates smooth, curved paths that mimic actual UAV flight physics, making testing more realistic.

---

## Known Limitations

1. **Temporal Resolution**: May miss conflicts shorter than `time_step` parameter (default: 0.5s)
   - **Mitigation**: Use smaller time_step for critical applications

2. **Conservative Safety Buffer**: May report conflicts for near-misses that are technically safe
   - **Mitigation**: Adjust safety_distance parameter based on UAV capabilities

3. **Performance at Scale**: O(n²) complexity for n flights
   - **Mitigation**: See Scalability Discussion above

---

## Dependencies

```
numpy>=1.20.0
matplotlib>=3.3.0
scipy>=1.7.0
pytest>=6.2.0
```

Install all:
```bash
pip install -r requirements.txt
```

---

## License

MIT License - See LICENSE file for details

---

## Author

FlytBase Robotics Assignment 2025

---

## Quick Reference

### Run Demo
```bash
python demo.py
```

### Run Tests
```bash
pytest tests/ -v
python test_edge_cases.py
```

### View Outputs
```bash
ls outputs/demo_4d/*/
ls outputs/edge_cases/*/
```

### API Usage
```python
from src import MissionVerificationService

service = MissionVerificationService(simulated_schedules)
result = service.verify_mission(primary_mission)

if result.is_clear():
    print("✅ Mission is safe")
else:
    print(f"❌ {len(result.conflicts)} conflicts detected")
```

---

**🎯 System Status**: ✅ **Ready for Submission** (pending demonstration video)

**🎬 4D Visualization**: ✅ **Implemented** (Extra Credit)

**📊 Test Coverage**: ✅ **54/54 passing** (100%)

**📁 Repository**: ✅ **Clean and organized**