# Frontier Exploration Integration Guide

## Overview

This guide shows how to integrate **frontier_exploration** (from GitHub: https://github.com/adrian-soch/frontier_exploration.git) with your OMPL path planner in a Unity simulation environment, using multi-slice 2D maps based on drone height.

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     Unity Simulation                         │
│              (publishes /map_3d, /drone_height)              │
└────────────────────────────┬────────────────────────────────┘
                             │
                    ┌────────┴────────┐
                    ▼                 ▼
        ┌──────────────────┐  ┌──────────────────┐
        │ frontier_explorer│  │ exploration_node │
        │  (detect 2D      │  │  (multi-slice    │
        │   frontiers)     │  │   + 3D targets)  │
        └────────┬─────────┘  └────────┬─────────┘
                 │                     │
                 └─────────┬───────────┘
                           ▼
                  ┌─────────────────────┐
                  │   OMPL Planner      │
                  │ (path planning to   │
                  │  frontier targets)  │
                  └─────────┬───────────┘
                            │
                            ▼
                  ┌─────────────────────┐
                  │  Path execution     │
                  │  (send to controller)
                  └─────────────────────┘
```

## File Structure

New files created in `ros2_ws/src/path_planning_pkg/`:

```
path_planning_pkg/
├── src/
│   ├── exploration_node.cpp          ✨ NEW - Main exploration node
│   ├── multi_slice_logic.cpp         ✨ NEW - Multi-slice 2D map generation
│   ├── frontier_to_ompl.cpp          ✨ NEW - 2D→3D frontier conversion
│   └── pathplanner.cpp               (existing)
├── include/
│   ├── multi_slice_logic.hpp         ✨ NEW - Multi-slice header
│   ├── frontier_to_ompl.hpp          ✨ NEW - Frontier→OMPL header
│   └── pathplanner.h                 (existing)
├── launch/
│   └── exploration.launch.py         ✨ NEW - Launch file
├── config/
│   └── exploration_params.yaml       ✨ NEW - Configuration
├── CMakeLists.txt                    📝 UPDATED - Added dependencies
└── package.xml                       📝 UPDATED - Added dependencies
```

---

## Step-by-Step Integration

### **Step 1: Verify frontier_exploration is present**

```bash
# Check if frontier_exploration is in your workspace
ls -la ros2_ws/src/frontier_exploration/

# Expected structure:
# frontier_exploration/          (main package)
# frontier_interfaces/           (service definitions)
# learned_frontier_detector/     (ML component - optional)
```

**Status:** ✅ Already in your workspace as a git submodule

---

### **Step 2: Build the packages**

```bash
cd /home/bk/Autonomous_Systems/Autonomous-Systems-Sub-Terrain-Challenge-GR04/ros2_ws

# Clean previous builds (optional)
rm -rf build install log

# Build all packages including new exploration_node
colcon build --symlink-install

# Or build specific packages in order:
colcon build --packages-select frontier_interfaces
colcon build --packages-select frontier_exploration
colcon build --packages-select path_planning_pkg
```

**Expected output:**
```
Finished `colcon build` in X.XXs
  Packages: 13
  Successes: 13
```

---

### **Step 3: Source the environment**

```bash
source install/setup.bash
```

---

### **Step 4: Launch the exploration system**

#### **Option A: Using the launch file (Recommended)**

```bash
# Default parameters
ros2 launch path_planning_pkg exploration.launch.py

# Or with custom parameters
ros2 launch path_planning_pkg exploration.launch.py \
  exploration_height:=3.5 \
  height_margin:=4.0 \
  frontier_rank:=0
```

#### **Option B: Run nodes individually**

```bash
# Terminal 1: Frontier explorer node
ros2 run frontier_exploration classical_frontier_detector \
  --ros-args -p occupancy_map_msg:=/map_3d

# Terminal 2: Exploration node
ros2 run path_planning_pkg exploration_node \
  --ros-args \
    -p exploration_height:=2.0 \
    -p height_margin:=3.0 \
    -p map_topic:=/map_3d
```

---

### **Step 5: Integrate with your Unity simulation**

Your Unity simulation must publish:

#### **Topic 1: 3D Occupancy Grid Map**
```
Topic:    /map_3d
Message:  nav_msgs/OccupancyGrid
Frequency: 1-10 Hz

Fields:
  header.frame_id: "map"
  info.width:      grid width (cells)
  info.height:     grid height (cells)
  info.resolution: cell resolution (meters/cell)
  info.origin.position: (x, y, z) of map origin
  data:            int8[] occupancy values (-1: unknown, 0-100: occupied)
```

**Example publisher in C# (Unity):**
```csharp
// Pseudocode for Unity
var occupancyGrid = new OccupancyGrid();
occupancyGrid.info.width = mapWidth;
occupancyGrid.info.height = mapHeight;
occupancyGrid.info.resolution = 0.1f; // 10cm resolution
occupancyGrid.info.origin.position = new Point(x_origin, y_origin, z_origin);

// Fill occupancy data from your voxel grid
for (int i = 0; i < mapData.Length; i++) {
    occupancyGrid.data[i] = mapData[i]; // 0-100
}

publisher.Publish(occupancyGrid);
```

#### **Topic 2: Drone Height**
```
Topic:    /drone_height
Message:  std_msgs/Float64
Frequency: 10 Hz

Fields:
  data: current drone Z-coordinate (meters)
```

---

### **Step 6: Connect to OMPL Path Planner**

The `exploration_node` publishes frontier targets on:

```
Topic:    /frontier_target
Message:  geometry_msgs/PoseStamped
```

Subscribe to this topic in your OMPL planner:

```cpp
// Example in your OMPL planner node
auto frontier_target_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
  "frontier_target", 10,
  [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // Send goal to OMPL planner
    ompl_planner->setGoal(msg->pose.position.x, 
                          msg->pose.position.y, 
                          msg->pose.position.z);
  });
```

---

## Configuration Parameters

Edit `ros2_ws/src/path_planning_pkg/config/exploration_params.yaml`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `exploration_height` | 2.0 | Target height for frontier goals (meters) |
| `height_margin` | 3.0 | ±margin around drone to consider for slices (meters) |
| `slice_thickness` | 0.5 | Vertical thickness of each 2D slice (meters) |
| `map_topic` | `/map_3d` | Topic name for occupancy grid |
| `drone_height_topic` | `/drone_height` | Topic name for drone height |
| `frontier_rank` | 0 | Which frontier to select (0=closest, 1=2nd, ...) |
| `max_frontier_cost` | 50.0 | Max distance to accept frontier (meters) |
| `min_exploration_height` | 0.5 | Minimum height constraint (meters) |
| `max_exploration_height` | 10.0 | Maximum height constraint (meters) |

---

## Multi-Slice Logic Explanation

The **multi-slice logic** works as follows:

1. **Receive 3D occupancy data** from your Unity simulation
2. **Extract horizontal slices** at different heights relative to drone position
   - Slice 1: [drone_z - margin, drone_z - margin + thickness]
   - Slice 2: [drone_z - margin + thickness, drone_z - margin + 2×thickness]
   - ... continues until drone_z + margin
3. **Project 3D frontier** points to 3D coordinates:
   - Keep XY from frontier_exploration's 2D detection
   - Add Z coordinate = slice height
4. **Send 3D targets** to OMPL planner

**Visual Example:**
```
Height: 10m  ▢ ▢ ▢ (Slice 5)
             ▢ ▢ ▢
Height: 8m   ▢ ▢ ▢ (Slice 4)
             ▢ ▢ ▢
Height: 6m   ▢ ▢ ▢ (Slice 3) ← Drone at 5m
             ▢ ▢ ▢
Height: 4m   ▢ ▢ ▢ (Slice 2)
             ▢ ▢ ▢
Height: 2m   ▢ ▢ ▢ (Slice 1)

Each slice extracts a 2D frontier map at that height
```

---

## Troubleshooting

### **Issue: "frontier_pose service not available"**

**Solution:**
```bash
# Check if frontier_explorer is running
ros2 node list | grep frontier

# If not running, start it:
ros2 run frontier_exploration classical_frontier_detector
```

### **Issue: "Waiting for map data..."**

**Solution:**
```bash
# Check if map is being published
ros2 topic list | grep map_3d
ros2 topic echo /map_3d --once
```

### **Issue: frontier targets not being published**

**Solution:**
```bash
# Check exploration_node logs
ros2 run path_planning_pkg exploration_node

# Monitor published topics
ros2 topic echo /frontier_target
ros2 topic echo /exploration_slices
```

### **Issue: Build errors (missing dependencies)**

**Solution:**
```bash
# Install missing packages
sudo apt-get install ros-humble-nav-msgs ros-humble-visualization-msgs

# Or use rosdep
rosdep install --from-paths src --ignore-src -r -y
```

---

## Testing the Integration

### **Test 1: Verify all nodes start**

```bash
ros2 launch path_planning_pkg exploration.launch.py &
sleep 3
ros2 node list
```

Expected output:
```
/exploration_node
/frontier_explorer
```

### **Test 2: Verify service works**

```bash
# Test frontier_exploration service directly
ros2 service call /frontier_pose frontier_interfaces/srv/FrontierGoal "{goal_rank: 0}"
```

Expected response:
```
requester: making request: frontier_interfaces.srv.FrontierGoal_Request(goal_rank=0)
response: frontier_interfaces.srv.FrontierGoal_Response(goal_pose=geometry_msgs.msg.PoseStamped(...))
```

### **Test 3: Publish test map**

```bash
# Create a simple test publisher
ros2 pub /map_3d nav_msgs/OccupancyGrid "{
  header: {frame_id: 'map'},
  info: {width: 100, height: 100, resolution: 0.1, origin: {position: {x: 0, y: 0, z: 0}}}
}"
```

---

## Next Steps

1. **Implement OMPL integration**: Subscribe to `/frontier_target` in your OMPL planner
2. **Tune parameters**: Adjust `exploration_height`, `height_margin`, and `slice_thickness`
3. **Add visualizations**: Use RViz to visualize:
   - Occupancy grids: `/map_3d`, `/exploration_slices`
   - Frontiers: `/f_markers` (from frontier_explorer)
   - Targets: `/frontier_targets_markers`
4. **Test with Unity**: Run your simulation and verify map publishing

---

## Key Classes & Methods

### `ExplorationNode`
- `mapCallback()` - Receives 3D occupancy grid
- `heightCallback()` - Updates drone height
- `explorationLoop()` - Main exploration cycle (2Hz)
- `frontierResponseCallback()` - Processes frontier service response
- `publishFrontierTarget()` - Sends 3D target to OMPL

### `MultiSliceLogic`
- `generateHeightSlices()` - Extract 2D slices from 3D grid
- `convertTo3DFrontiers()` - Add Z coordinate to 2D points
- `extract2DSlice()` - Project 3D data to 2D at height range

### `FrontierToOMPL`
- `convertFrontierToTarget()` - Add height to 2D frontier pose
- `rankTargets()` - Sort by distance cost
- `filterByHeight()` - Constrain targets to height range
- `calculateDistance()` - 3D Euclidean distance

---

## Dependencies Added

```xml
<depend>nav_msgs</depend>
<depend>geometry_msgs</depend>
<depend>visualization_msgs</depend>
<depend>frontier_interfaces</depend>
```

All are standard ROS2 packages available via `apt` or `rosdep`.

---

## Support & Resources

- **Frontier Exploration GitHub**: https://github.com/adrian-soch/frontier_exploration
- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **OMPL Documentation**: https://ompl.kavrakilab.org/
- **RViz Guide**: https://wiki.ros.org/rviz
