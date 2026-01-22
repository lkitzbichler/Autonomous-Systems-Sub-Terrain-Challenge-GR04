# [Autonomous Systems](https://sharelatex.tum.de/read/wchjfkvjgrcb#760f11): <br> Project Sub-Terrain Challenge (WS2025/26)

# Contents
- [1. Project Planning](#1-project-planning)
  - [1.1 Team](#11-team)
  - [1.2 Requirements](#12-requirements)
  - [1.3 Working Packages](#13-working-packages)
  - [1.4 Project Plan](#14-project-plan)
- [2. Setup & Structure](#2-setup--structure)
    - [2.1 Setup Guide](#21-setup-guide)
      - [2.1.1 Prerequisites](#210-prerequisites)
      - [2.1.1 Clone Repository](#211-clone-repository)
      - [2.1.2 Install & Setup VSCode](#212-install--setup-vscode)
      - [2.1.3 Install missing packages](#213-install-all-missing-packages)
      - [2.1.4 Install & Setup ROS2 jazzy](#214-install--setup-ros2-jazzy)
      - [2.1.5 Download & Move Simulation](#215-download--move-simulation)
      - [2.1.6 Build Code](#216-build-code)
      - [2.1.7 Run Everything](#217-run-everything)
    - [2.2 Structure Planning](#22-structure-planning)
        - [2.2.1 Ros2-Nodes](#221-ros2-nodes)
        - [2.2.2 Ros2-Packages](#222-ros2-packages)
- [3. Methodology](#3-methodology)
- [4. Results](#4-results)
- [Literature](#literature)

---

# 1. Project Planning

<details>
<summary> Show Sub chapters </summary>

## 1.1 Team

<details>

| ID | Last Name | First Name | Matriculation number | Github Name |
|---:|-----------|------------|----------------------|-------------|
| 01 | Heller    | Leo        | N/A                  | LeoHeller   |
| 02 | Kitzbichler | Leon | N/A | lkitzbichler |
| 03 | Kristandra | Brian | N/A | N/A |
| 04 | Thimm | Dominik | N/A | dominik-thimm |
| 05 | Waeger | Sebastian | N/A | 03807001 |

</details>

## 1.2 Requirements

<details>



</details>

## 1.3 Working Packages

<details>



</details>

## 1.4 Project Plan

<details>



</details>

</details>

---
# 2. Setup & Structure

<details>
<summary> Show Sub chapters </summary>

## 2.1 Setup Guide

<details>

### 2.1.0 Prerequisites

<details>

```text
OS      :         Ubuntu 24.04
ROS     :         ROS2 Jazzy
ROS-DIR :         /opt/ros/jazzy
```

---

For use of git, successfull compilation of c++ and the use of python the following packages have to be installed:

```bash
sudo apt install -y git git-lfs curl build-essential
```

Install required ros2 packages
```bash
sudo apt install ros-jazzy-octomap-server ros-jazzy-pcl-ros ros-jazzy-depth-image-proc
```
</details>

### 2.1.1 Clone Repository

### 2.1.2 Install & Setup VSCode

### 2.1.3 Install all missing packages 

### 2.1.4 Install & Setup ROS2 jazzy

### 2.1.5 Download & Move Simulation

### 2.1.6 Build Code

<details>

Bash file should be executable already.
If not run:

```bash
chmod +x build.bash
```

When willing to finally build, run:

```bash
./build.bash
```

</details>

### 2.1.7 Run Everything

<details>

Bash file should be executable already.
If not run:

```bash
chmod +x run.bash
```

When willing to run the simulation and rest, run:

```bash
./run.bash
```

</details>

</details>

## 2.2 Structure Planning

```mermaid
flowchart LR
  %% --- basic_waypoint_pkg ---
  subgraph basic_waypoint_pkg
    Planner["planner (basic_waypoint_node)"]
    RViz[(RViz)]
  end

  %% --- mav_trajectory_generation ---
  subgraph mav_trajectory_generation
    Sampler["trajectory_sampler_node"]
    Ext1[(service)]
    Ext2[(service)]
  end

  %% --- controller_pkg ---
  subgraph controller_pkg
    Controller["controller_node"]
  end

  %% --- unity_bridge ---
  subgraph unity_bridge
    UnityState["unity_state"]
    W2U["w_to_unity"]
    Unity[(Unity sim)]
  end

  %% --- connections ---
  Planner -- "trajectory (mav_planning_msgs/PolynomialTrajectory4D)" --> Sampler
  Planner -- "trajectory_markers (visualization_msgs/MarkerArray)" --> RViz

  Sampler -- "command/trajectory (trajectory_msgs/MultiDOFJointTrajectory)" --> Controller
  Sampler -. "stop_sampling (std_srvs/Empty)" .-> Ext1
  Sampler -. "back_to_position_hold (std_srvs/Empty)" .-> Ext2

  UnityState -- "current_state (nav_msgs/Odometry)" --> Controller
  Controller -- "rotor_speed_cmds (mav_msgs/Actuators)" --> W2U

  W2U -. "UDP 12346" .-> Unity
  Unity -. "TCP 12347" .-> UnityState

```

### 2.2.1 Ros2-Nodes

<details>


```mermaid
flowchart LR
  %% Packages
  subgraph basic_waypoint_pkg
    planner[basic_waypoint_node]
  end

  subgraph mav_trajectory_generation
    sampler[trajectory_sampler_node]
  end

  subgraph controller_pkg
    controller[controller_node]
  end

  subgraph simulation
    unity_state[unity_state]
    unity_ros[unity_ros]
    corruptor[state_estimate_corruptor_node]
    w_to_unity[w_to_unity]
    unity_sim[(Unity sim)]
  end

  %% Core control loop
  unity_state -- "/current_state" --> planner
  unity_state -- "/current_state" --> controller
  planner -- "/trajectory" --> sampler
  sampler -- "/command/trajectory" --> controller
  controller -- "/rotor_speed_cmds" --> w_to_unity
  w_to_unity -. "UDP 12346" .-> unity_sim
  unity_sim -. "TCP 12347" .-> unity_state

  %% Sensor stream + state corruption
  unity_ros -- "/true_pose" --> corruptor
  unity_ros -- "/true_twist" --> corruptor
  corruptor -- "/pose_est" --> pose_est_topic[(Pose Estimation Topics)]
  corruptor -- "/twist_est" --> twist_est_topic[(Twist Estimation Topics)]
  corruptor -- "/current_state_est" --> state_est_topic[(Odometry Estimate)]

  %% Unity sensor topics (remapped)
  unity_ros -- "/realsense/rgb/left_image_raw" --> cam_left[(Camera Topics)]
  unity_ros -- "/realsense/rgb/right_image_raw" --> cam_right[(Camera Topics)]
  unity_ros -- "/realsense/depth/image" --> depth[(Depth Topics)]
  unity_ros -- "/interpolate_imu/imu" --> imu[(IMU Topic)]

```



</details>

### 2.2.2 Ros2-Packages

<details>



</details>

### 2.2.3 Ros2-Services

<details>

```mermaid
flowchart LR
  sampler[trajectory_sampler_node]
  stop_srv["/stop_sampling (std_srvs/Empty)"]
  hold_srv["/back_to_position_hold (std_srvs/Empty)"]

  sampler -- "server" --> stop_srv
  sampler -. "client" .-> hold_srv

```

</details>

</details>


---
# 3. Methodology

<details>



</details>

---
# 4. Results

<details>



</details>

---
# Literature

<details>



</details>