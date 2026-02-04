# Quick Start: Frontier Exploration + OMPL

## 1. Build (5 minutes)

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## 2. Launch (1 command)

```bash
ros2 launch path_planning_pkg exploration.launch.py
```

This starts:
- ✅ `frontier_explorer` - Detects 2D frontiers from occupancy maps
- ✅ `exploration_node` - Converts 2D frontiers to 3D targets for OMPL

## 3. Publish map from Unity

Your Unity simulation must continuously publish:

```
Topic: /map_3d (nav_msgs/OccupancyGrid)
Topic: /drone_height (std_msgs/Float64)
```

## 4. Subscribe to targets in OMPL

Listen for 3D frontier targets:

```
Topic: /frontier_target (geometry_msgs/PoseStamped)
```

---

## Architecture Flow

```
Unity Simulation                (sends 3D map + drone height)
         │
         ▼
Frontier Explorer              (detects 2D frontiers)
         │
         ▼
Exploration Node               (adds height, creates 3D targets)
         │
         ▼
OMPL Path Planner             (plans path to target)
         │
         ▼
Controller & Execution        (execute path)
```

---

## File Locations

- **Main node**: `ros2_ws/src/path_planning_pkg/src/exploration_node.cpp`
- **Multi-slice logic**: `ros2_ws/src/path_planning_pkg/src/multi_slice_logic.cpp`
- **Frontier converter**: `ros2_ws/src/path_planning_pkg/src/frontier_to_ompl.cpp`
- **Config**: `ros2_ws/src/path_planning_pkg/config/exploration_params.yaml`
- **Launch**: `ros2_ws/src/path_planning_pkg/launch/exploration.launch.py`

---

## ROS2 Topics

### Input Topics:
- `/map_3d` (nav_msgs/OccupancyGrid) - Your Unity 3D occupancy grid
- `/drone_height` (std_msgs/Float64) - Current drone Z height

### Output Topics:
- `/frontier_target` (geometry_msgs/PoseStamped) - **3D frontier goal for OMPL**
- `/frontier_targets_markers` (visualization_msgs/MarkerArray) - Rviz visualization
- `/exploration_slices` (nav_msgs/OccupancyGrid) - 2D slices being analyzed
- `/f_markers` (visualization_msgs/Marker) - Frontier markers from detector
- `/f_map` (nav_msgs/OccupancyGrid) - Processed frontier map

### Service:
- `/frontier_pose` (frontier_interfaces/srv/FrontierGoal) - Called internally

---

## Key Parameters (Edit in `exploration_params.yaml`)

```yaml
exploration_height: 2.0        # Height of frontier goals
height_margin: 3.0             # ±3m around drone for multi-slice analysis
slice_thickness: 0.5           # Each slice is 0.5m thick
map_topic: "/map_3d"           # Where to listen for maps
frontier_rank: 0               # Pick closest frontier (0=closest)
max_frontier_cost: 50.0        # Don't explore targets >50m away
```

---

## Multi-Slice Logic

**What it does:**
1. Receives your 3D occupancy grid from Unity
2. Extracts 2D horizontal slices at different heights
3. For each slice, frontier_explorer detects 2D frontiers
4. Converts 2D frontier points to 3D by adding the slice height

**Example:**
- Drone at height 2m
- `height_margin = 3.0`, `slice_thickness = 0.5`
- Generates 6 slices: [−1m, −0.5m], [−0.5m, 0m], [0m, 0.5m], ..., [4.5m, 5m]
- Each slice gets separate 2D frontier detection
- XY coordinates from frontier + Z from slice height = 3D target

---

## Verify Everything Works

```bash
# In one terminal:
ros2 launch path_planning_pkg exploration.launch.py

# In another, watch the targets:
ros2 topic echo /frontier_target

# In another, publish a test map (from your Unity sim or test script):
ros2 pub /map_3d nav_msgs/OccupancyGrid \
  "{header: {frame_id: 'map'}, info: {width: 100, height: 100, resolution: 0.1, origin: {position: {}}}}" \
  -r 1

# Should see frontier targets appear in the first terminal
```

---

## Next: Integrate with OMPL

In your OMPL planner node, add this subscription:

```cpp
auto target_sub = node->create_subscription<geometry_msgs::msg::PoseStamped>(
  "frontier_target", 10,
  [&](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // msg->pose.position = 3D frontier target
    ompl_planner->plan(msg->pose.position);
  });
```

---

## Troubleshooting

| Issue | Check |
|-------|-------|
| "frontier_pose service not available" | Is `frontier_explorer` running? |
| "Waiting for map data..." | Is `/map_3d` being published from Unity? |
| "No targets published" | Check logs: `ros2 topic echo /frontier_target` |
| Build fails | Run: `sudo apt-get install ros-humble-nav-msgs` |

---

## For Detailed Info

See: `FRONTIER_EXPLORATION_INTEGRATION.md`
