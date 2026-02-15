# Path Planning Interface Contract (Step 1)

This document defines the integration contract for `path_planning_pkg` with the
existing project architecture.

Scope of this step:
- Define interfaces and behavior contract only.
- No planner algorithm implementation yet.

Out of scope of this step:
- Graph building, frontier extraction, path planning logic.
- Changes to `statemachine_pkg/msg/*` message definitions.

## 1. Node Identity and Runtime Role

- ROS node name: `path_planner`
- Package: `path_planning_pkg`
- Executable: `pathplanner_node`
- Mission role:
  - Receives mission commands from the statemachine.
  - Plans exploration/return-home trajectories.
  - Publishes trajectory output and heartbeat/events.

## 2. Topics and Message Contract

### 2.1 Inputs

1. `statemachine/cmd` (`statemachine_pkg/msg/Command`)
- Required
- Must ignore messages where `msg.target != "path_planner"`.
- Relevant commands for planner:
  - `START` -> enter exploration mode.
  - `SWITCH_TO_EXPLORE` -> optional alias for exploration mode.
  - `RETURN_HOME` -> enter return-home mode.
  - `HOLD` -> stop planning/trajectory updates.
  - `ABORT` -> enter fail-safe stop mode.
- Non-relevant commands are ignored and logged at debug/info level.
- Commands with stale/out-of-order timestamps are ignored.

2. `statemachine/state` (`std_msgs/msg/String`)
- Required for phase-aware behavior.
- Planner uses this topic to gate when graph recording is allowed.
- Recording start condition:
  - Start recording only once mission state becomes `TRAVELLING`
    (after first checkpoint reached).
- Recording must not include TAKEOFF-only path.

3. `current_state_est` (`nav_msgs/msg/Odometry`)
- Required for planner progress evaluation and goal checks.
- Planning frame consistency required (see Section 4).

4. `octomap_binary` (`octomap_msgs/msg/Octomap`)
- Required for map queries and collision checks.
- If unavailable/invalid, planner must remain safe and report condition.

### 2.2 Outputs

1. `heartbeat` (`statemachine_pkg/msg/Answer`)
- Periodic heartbeat from planner.
- `node_name` must be `path_planner`.
- `state` values:
  - `RUNNING` for normal operation (active or idle).
  - `DONE` only when planner has completed a requested mission phase
    (`ExplorationDone` or `ReturnHomeReached`).
- `info` field is used as event/status code (see Section 3).

2. `trajectory` (`mav_planning_msgs/msg/PolynomialTrajectory4D`)
- Planned trajectory output for trajectory sampler/controller pipeline.
- Frame must be `world`.

3. Debug topics (planned, optional in later steps)
- `path_planner/graph_markers` (`visualization_msgs/msg/MarkerArray`)
- `path_planner/frontier_markers` (`visualization_msgs/msg/MarkerArray`)
- `path_planner/current_plan` (`nav_msgs/msg/Path`)
- `path_planner/stats` (custom or string-based status topic)

### 2.3 Internal Map Query API (planner-internal)

The planner provides a reusable map adapter interface:

- `is_free(p)`
- `is_occupied(p)`
- `is_unknown(p)`
- `raycast(origin, target)`
- `clearance(p)`

These checks run in planning frame coordinates and apply safety inflation.

### 2.4 Graph Core API (planner-internal)

The planner maintains a persistent topological graph:

- Node fields:
  - `id, position, stamp, status, frontier_score, loop_id, is_transit`
- Edge fields:
  - `from_id, to_id, length_m, cost, valid`

Core operations:

- `add_or_merge_node(point, merge_radius)`
- `upsert_edge(from_id, to_id)`
- `find_nearest_node(point, radius)`
- `find_nearby_nodes(point, radius)`

Deterministic tie-break:
- nearest node by distance, then lower node id.

## 3. Event and Status Mapping (without new messages)

Until dedicated planner event messages exist, planner events are encoded via
`Answer.info` with stable strings.

### 3.1 RUNNING states (`Answer.state = RUNNING`)

- `RUNNING_IDLE`
- `RUNNING_TRANSIT_RECORD`
- `RUNNING_EXPLORE`
- `RUNNING_BACKTRACK`
- `RUNNING_RETURN_HOME`
- `RUNNING_HOLD`

### 3.2 DONE events (`Answer.state = DONE`)

- `DONE_EXPLORATION_COMPLETE`
- `DONE_RETURN_HOME_REACHED`

### 3.3 Fault/attention events (`Answer.state = RUNNING`, info code)

- `EVENT_BRANCH_DETECTED`
- `EVENT_LOOP_CLOSED`
- `EVENT_STUCK`
- `EVENT_NEED_HELP`

Note:
- Current statemachine logic only consumes `DONE` from planner state, not the
  `info` code. `info` is still required for logs/debug determinism.

## 4. Frames, Time, and Consistency Rules

1. Planning frame:
- Primary planning frame is `world`.

2. Input validation:
- Reject or warn if `frame_id` is empty or inconsistent with planner frame.
- Validate timestamps; detect stale data (parameterized timeout later).

3. Output consistency:
- Trajectory and debug outputs must use `world`.

## 5. Planner Operating Modes (internal contract)

Internal mode state machine for `path_planner`:

1. `IDLE`
- Waiting for `START` or `RETURN_HOME`.
- Graph recording disabled.

2. `TRANSIT_RECORD`
- Passive breadcrumb recording mode.
- Activated by `statemachine/state == TRAVELLING`.
- Purpose: capture the valid transit path from first checkpoint toward cave.
- No exploration policy decisions in this mode.

3. `EXPLORE`
- Autonomous exploration phase.

4. `BACKTRACK`
- Active when no local progress/frontier is possible and unvisited nodes exist.

5. `RETURN_HOME`
- Global route to home/takeoff node.

6. `HOLD`
- Planner suspended by command.

7. `ABORTED`
- Fail-safe stop; no new trajectory commands.

## 6. Determinism Rules

- No uncontrolled random behavior.
- If tie in score/cost, choose by:
  1. lower total cost
  2. lower Euclidean distance
  3. lower node id

## 7. Parameter Contract (to be implemented incrementally)

All runtime behavior must be tunable by ROS parameters. Planned keys:

- `topics.command_topic` (default: `statemachine/cmd`)
- `topics.state_topic` (default: `statemachine/state`)
- `topics.heartbeat_topic` (default: `heartbeat`)
- `topics.odom_topic` (default: `current_state_est`)
- `topics.map_topic` (default: `octomap_binary`)
- `topics.trajectory_topic` (default: `trajectory`)

- `time.heartbeat_period_sec` (default: `1.0`)
- `time.input_timeout_sec` (default: `1.0`)
- `time.replan_period_sec` (default: `0.5`)

- `frames.planning_frame` (default: `world`)

- `graph.node_spacing_m` (default: `10.0`)
- `graph.transit_node_spacing_m` (default: `10.0`)
- `graph.merge_radius_m` (default: `3.0`)
- `graph.ahead_query_dist_m` (default: `12.0`)

- `safety.inflation_m` (default: `1.0`)
- `safety.min_clearance_m` (default: `1.5`)
- `safety.clearance_step_m` (default: `0.5`)
- `safety.clearance_search_max_m` (default: `8.0`)
- `safety.max_vertical_step_m` (default: `2.0`)

- `explore.branch_min_len_m` (default: `6.0`)
- `explore.dead_end_min_progress_m` (default: `2.0`)

## 8. Fail-Safe Rules

If any critical dependency is invalid (`TF/map/odom/planner-fail`):

1. Stop publishing new trajectories.
2. Publish heartbeat with `RUNNING` + `EVENT_STUCK` or `EVENT_NEED_HELP`.
3. Stay in safe mode (`HOLD`/`ABORTED`) until a new valid command is received.

## 9. Step-1 Acceptance (recommended test checklist)

This acceptance is intentionally a test recommendation, as requested.

1. Launch stack and ensure planner node name is `path_planner`.
2. Verify no graph recording while mission state is not `TRAVELLING`.
3. Switch mission state to `TRAVELLING` (real transition or controlled test).
4. Verify planner enters `TRANSIT_RECORD` behavior and starts breadcrumb capture.
5. Publish command `START` targeted to `path_planner`.
6. Verify heartbeat is published to `heartbeat` with:
- `node_name = "path_planner"`
- `state = RUNNING`
- `info = RUNNING_EXPLORE` (or temporary mapped state during stepwise rollout)
7. Publish `HOLD` and verify heartbeat transitions to `RUNNING_HOLD`.
8. Publish `RETURN_HOME` and verify planner mode heartbeat switches accordingly.
9. Publish `ABORT` and verify fail-safe state (`RUNNING` + fault/hold info code).

Expected result:
- Contract behavior is reproducible and deterministic for command handling and
  heartbeat event encoding.
