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

For use of git, successfull compilation of c++ and the use of python the following packages have to be installed:

```bash
sudo apt install -y git git-lfs curl build-essential
```

</details>

### 2.1.1 Clone Repository

### 2.1.2 Install & Setup VSCode

### 2.1.3 Install all missing packages 

### 2.1.4 Install & Setup ROS2 jazzy

### 2.1.5 Download & Move Simulation

### 2.1.6 Build Code

### 2.1.7 Run Everything

</details>

## 2.2 Structure Planning
### 2.2.1 Ros2-Nodes

<details>

```mermaid 
stateDiagram-v2
  [*] --> Idle
  Idle --> Working: start
  Working --> Idle: finish
  Working --> Error: fail
  Error --> Idle: reset
  Idle --> [*]
```

</details>

### 2.2.2 Ros2-Packages

<details>



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