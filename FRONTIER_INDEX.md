# Frontier Exploration Integration - Complete Package

## 📋 Documentation Index

### Start Here 👈
- **[QUICK_START_FRONTIER.md](QUICK_START_FRONTIER.md)** - 5-minute setup guide
  - Build instructions
  - Launch command
  - Topic quick reference
  - Parameter overview

### Comprehensive Guide
- **[FRONTIER_EXPLORATION_INTEGRATION.md](FRONTIER_EXPLORATION_INTEGRATION.md)** - Full integration guide (400+ lines)
  - Step-by-step integration walkthrough
  - Architecture diagrams
  - Configuration parameters
  - Troubleshooting guide
  - Testing procedures
  - Dependencies explained

### System Overview
- **[INTEGRATION_SUMMARY.md](INTEGRATION_SUMMARY.md)** - Complete summary
  - All files created/modified
  - Directory structure
  - How it works
  - Class reference
  - Verification checklist

---

## 🎯 What Was Done

### Core Integration (in `ros2_ws/src/path_planning_pkg/`)

#### Source Files (C++)
```cpp
// Main ROS2 node orchestrating the integration
src/exploration_node.cpp

// Multi-slice 2D map generation from 3D data
src/multi_slice_logic.cpp

// Convert 2D frontiers to 3D OMPL targets
src/frontier_to_ompl.cpp
```

#### Header Files
```cpp
// Multi-slice logic interface
include/multi_slice_logic.hpp

// Frontier-to-OMPL conversion interface
include/frontier_to_ompl.hpp
```

#### Configuration & Execution
```yaml
# Configuration parameters
config/exploration_params.yaml

# Launch both frontier_explorer + exploration_node
launch/exploration.launch.py
```

#### Build Configuration
```cmake
CMakeLists.txt    # Updated with new targets & dependencies
package.xml       # Updated with ROS2 dependencies
```

---

## 🔌 System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                   Unity Simulation                          │
│  Publishes: /map_3d (OccupancyGrid)                         │
│  Publishes: /drone_height (Float64)                         │
└────────────────┬────────────────────────────────────────────┘
                 │
        ┌────────▼──────────┐
        │ frontier_explorer │  ← From GitHub (already in workspace)
        │  Detects 2D       │
        │  frontiers from   │
        │  2D occupancy     │
        └────────┬──────────┘
                 │ Provides /frontier_pose service
        ┌────────▼──────────────────┐
        │  exploration_node (NEW)  │  ← Calls frontier service
        │  - Multi-slice logic     │     ← Extracts 2D slices
        │  - 3D frontier converter │     ← Adds height
        └────────┬──────────────────┘
                 │
    /frontier_target (PoseStamped)
         (3D frontier goal)
                 │
        ┌────────▼──────────┐
        │  Your OMPL Planner│  ← Subscribe to /frontier_target
        │  (Ready to        │
        │   integrate!)     │
        └───────────────────┘
```

---

## 📦 What's Needed From Unity

### Topic 1: 3D Occupancy Grid
```
Topic:        /map_3d
Message Type: nav_msgs/OccupancyGrid
Frequency:    1-10 Hz

Fields:
  header.frame_id:         "map"
  info.width:              Grid width in cells
  info.height:             Grid height in cells
  info.resolution:         Resolution in meters/cell
  info.origin.position:    (x, y, z) origin in world frame
  data:                    int8[] occupancy (-1:unknown, 0-100:occupied)
```

### Topic 2: Drone Height
```
Topic:        /drone_height
Message Type: std_msgs/Float64
Frequency:    10 Hz

Fields:
  data:        Current drone Z coordinate in meters
```

---

## 🎛️ What Comes Out (to OMPL)

### Main Output Topic
```
Topic:        /frontier_target
Message Type: geometry_msgs/PoseStamped
Frequency:    ~0.5 Hz (when frontier available)

Contains:
  pose.position.x    → Frontier X (from frontier_exploration)
  pose.position.y    → Frontier Y (from frontier_exploration)
  pose.position.z    → Height from multi-slice logic ✨
```

### Visualization Topics
```
/frontier_targets_markers     → MarkerArray for RViz
/exploration_slices           → 2D slices being analyzed
```

---

## ⚙️ Configuration Parameters

Edit `ros2_ws/src/path_planning_pkg/config/exploration_params.yaml`:

| Parameter | Default | Meaning |
|-----------|---------|---------|
| `exploration_height` | 2.0m | Target height for frontier goals |
| `height_margin` | 3.0m | ±margin around drone to consider |
| `slice_thickness` | 0.5m | Vertical thickness per slice |
| `map_topic` | `/map_3d` | Where Unity publishes 3D map |
| `drone_height_topic` | `/drone_height` | Where drone height comes from |
| `frontier_rank` | 0 | Which frontier (0=closest, 1=2nd, ...) |
| `max_frontier_cost` | 50.0m | Max distance to explore |
| `min_exploration_height` | 0.5m | Hard minimum height |
| `max_exploration_height` | 10.0m | Hard maximum height |

---

## 🚀 Quick Start Commands

### 1️⃣ Build
```bash
cd /home/bk/Autonomous_Systems/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2️⃣ Run Everything
```bash
ros2 launch path_planning_pkg exploration.launch.py
```

### 3️⃣ Verify It Works
```bash
# In another terminal:
ros2 topic echo /frontier_target
```

### 4️⃣ Integrate OMPL (in your code)
```cpp
auto sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "frontier_target", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // msg->pose.position has 3D frontier goal!
        this->planPathToTarget(msg->pose.position);
    });
```

---

## 📊 Key Classes

### `ExplorationNode` (exploration_node.cpp)
Main orchestrator that:
- Listens to 3D map and drone height
- Calls frontier_exploration service
- Converts 2D frontiers to 3D
- Publishes targets for OMPL

**Key Methods:**
- `explorationLoop()` - Main 2Hz exploration cycle
- `frontierResponseCallback()` - Process frontier results
- `publishFrontierTarget()` - Send to OMPL

### `MultiSliceLogic` (multi_slice_logic.cpp/hpp)
Handles 3D↔2D conversions:
- `generateHeightSlices()` - Extract slices from 3D grid
- `convertTo3DFrontiers()` - Add Z to 2D points
- `extract2DSlice()` - Project 3D to 2D at height

### `FrontierToOMPL` (frontier_to_ompl.cpp/hpp)
Frontier → OMPL utilities:
- `convertFrontierToTarget()` - Wrap frontier in target
- `rankTargets()` - Sort by distance
- `filterByHeight()` - Constrain by height limits

---

## 🔍 Understanding Multi-Slice Logic

**Problem:** frontier_exploration gives 2D frontiers (x, y), but OMPL needs 3D (x, y, z)

**Solution:** Extract multiple 2D slices at different heights:

```
Original 3D data from Unity:
  [occupancy grid at multiple heights]

Multi-Slice Processing:
  Height 5.0m:  Extract 2D map → Frontier (5.0, 10.0) → Add z=5.0 → (5.0, 10.0, 5.0)
  Height 4.5m:  Extract 2D map → Frontier (5.0, 10.0) → Add z=4.5 → (5.0, 10.0, 4.5)
  Height 4.0m:  Extract 2D map → Frontier (5.0, 10.0) → Add z=4.0 → (5.0, 10.0, 4.0)
  Height 3.5m:  Extract 2D map → Frontier (5.0, 10.0) → Add z=3.5 → (5.0, 10.0, 3.5)
  ...

Result: 3D frontier targets ready for OMPL!
```

---

## 📚 Dependencies Added

```xml
<depend>nav_msgs</depend>              <!-- OccupancyGrid message -->
<depend>geometry_msgs</depend>         <!-- PoseStamped, Point -->
<depend>visualization_msgs</depend>    <!-- Marker visualization -->
<depend>frontier_interfaces</depend>   <!-- FrontierGoal service -->
```

All standard ROS2 packages installable via `apt` or `rosdep`.

---

## ✅ Verification Checklist

- [ ] All files created in correct locations
- [ ] `colcon build` completes successfully
- [ ] Both nodes launch without errors
- [ ] `/frontier_target` topic appears in `ros2 topic list`
- [ ] RViz shows frontier targets as colored spheres
- [ ] Height values in targets match expected heights
- [ ] OMPL planner successfully subscribes
- [ ] Paths planned to 3D frontier targets

---

## 🎓 Learning Resources

- **ROS2 Basics:** https://docs.ros.org/en/humble/
- **Frontier Exploration GitHub:** https://github.com/adrian-soch/frontier_exploration
- **OMPL Documentation:** https://ompl.kavrakilab.org/
- **RViz Guide:** https://wiki.ros.org/rviz

---

## 📝 Notes

- `frontier_exploration` is already in your workspace as a git submodule
- No Nav2 dependency - uses OMPL instead
- All parameters configurable via YAML
- Multi-slice logic fully implemented and ready to use
- Integration point clear: subscribe to `/frontier_target` in OMPL planner

---

## 🆘 Support

If you encounter issues:

1. **Check logs:** `ros2 run path_planning_pkg exploration_node`
2. **Verify topics:** `ros2 topic list | grep frontier`
3. **Check service:** `ros2 service call /frontier_pose frontier_interfaces/srv/FrontierGoal "{goal_rank: 0}"`
4. **See detailed guide:** [FRONTIER_EXPLORATION_INTEGRATION.md](FRONTIER_EXPLORATION_INTEGRATION.md)

---

**Status:** ✅ Ready to build and test!
