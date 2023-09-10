
# UAV Simulation Repository

Welcome to the UAV Simulation Repository! This README provides a guide on how to run the available simulations and navigate through the codebase.

## Available Simulations

### A. Full Map for Performance Comparison

To run a full map simulation for performance comparison, follow these steps:

1. **Source the Workspace**
   
   Before starting, ensure you've sourced the workspace:
   ```bash
   source /path/to/workspace/setup.bash
   ```

2. **Start RViz Simulation**
   
   To initiate the RViz simulation:
   ```bash
   roslaunch thesis_uav_gpmp simulation.launch
   ```

3. **Choose an Optimizer**

   You can start any of the four available optimizers by launching one of the following commands:
   ```bash
   roslaunch thesis_uav_gpmp 3Dedt_batch.launch
   roslaunch thesis_uav_gpmp 3Dedt_isam.launch
   roslaunch thesis_uav_gpmp sdf_batch.launch
   roslaunch thesis_uav_gpmp sdf_isam.launch
   ```

4. **Generate and Execute Map**

   To start generating the map and executing it, call the ROS service:
   ```bash
   rosservice call /generate_cloud_and_reset_octomap
   ```
   Recall this service during online execution to visualize re-optimization in real-time.

### B. Map Exploration

1. **Source the Workspace**

   Ensure you've sourced the workspace:
   ```bash
   source /path/to/workspace/setup.bash
   ```

2. **Run Exploration Simulations**

   Choose and run one of the exploration simulations:
   ```bash
   roslaunch thesis_uav_gpmp simulation_explore_batch.launch
   ```
   or
   ```bash
   roslaunch thesis_uav_gpmp simulation_explore_isam.launch
   ```

3. **Control the Movement**

   Call the move service at your desired interval to control the UAV's movement:
   ```bash
   while true; do rosservice call /explore/move "data: true"; sleep 1.0; done
   ```

## Initial UAV Spawn Location

The initial location configuration for spawning each UAV can be found here:
```
src/multi_uav_simulator/multi_uav_simulator/config/initial_conditions.yaml
```
(Note: Ensure this configuration tallies with the initialization in each optimizer code.)

## Codebase

### Contributed Packages, Interfaces and Libraries

- **ROS Packages**:
  - `src/thesis_optimizer`
  - `src/thesis_uav_gpmp`

- **Controller Interfaces**:
  - `src/multi_uav_simulator/multi_uav_simulator/src/send_trajectory_explore.cpp`
  - `src/multi_uav_simulator/multi_uav_simulator/src/send_trajectory_robot2.cpp`
  - `src/multi_uav_simulator/multi_uav_simulator/src/send_trajectory_robot3.cpp`
  - `src/multi_uav_simulator/multi_uav_simulator/src/send_trajectory_robot4.cpp`
  - `src/multi_uav_simulator/multi_uav_simulator/src/send_trajectory.cpp`

- **GPMP Libraries**:
  - `gpmp2/planner/ISAM2TrajOptimizer-inl.h`
  - `gpmp2/planner/ISAM2TrajOptimizer.h`
  - `gpmp2/planner/ISAM2TrajOptimizerNew-inl.h`
  - `gpmp2/planner/ISAM2TrajOptimizerNew.h`
  - `gpmp2/planner/BatchTrajOptimizerNew.h`
  - `gpmp2/planner/BatchTrajOptimizerNew.cpp`
  - `gpmp2/planner/BatchTrajOptimizerNew-inl.h`
  - `gpmp2/planner/BatchTrajOptimizer.h`
  - `gpmp2/planner/BatchTrajOptimizer.cpp`
  - `gpmp2/planner/BatchTrajOptimizer-inl.h`
  - `gpmp2/obstacle/ObstacleSDFFactorPR3DNew.h`
  - `gpmp2/obstacle/ObstacleSDFFactorPR3D.h`
  - `gpmp2/obstacle/ObstacleSDFFactorNew-inl.h`
  - `gpmp2/obstacle/ObstacleSDFFactorNew.h`
  - `gpmp2/obstacle/ObstacleSDFFactorGPPR3DNew.h`
  - `gpmp2/obstacle/ObstacleSDFFactorGPPR3D.h`
  - `gpmp2/obstacle/ObstacleSDFFactorGPNew.h`
  - `gpmp2/obstacle/ObstacleSDFFactorGPNew-inl.h`
  - `gpmp2/obstacle/ObstacleCostNew.h`
  - `gpmp2/kinematics/PointRobot3DModel.h`
  - `gpmp2/kinematics/PointRobot3D.h`
  - `gpmp2/kinematics/PointRobot3D.cpp`

### Required Libraries

Ensure you have the following libraries installed:

- GTSAM (version: gtsam-release-4.2a8)
- GPMP2 (version: gpmp2-gtsam4.2)
- PCL
- Octomap 
- Dynamic 3D EDT 

### Forked UAV Controller 

The UAV controller is forked from the [Multi UAV simulator](https://github.com/malintha/multi_uav_simulator#mavswarm-a-lightweight-multi-aerial-vehicle-simulator).

### Supported ROS Version

This repository supports ROS 1 Noetic.

## Demonstration

A video demonstrating the algorithm in action is available. Please check the provided link or file to view it.

---

Thank you for exploring this repository. Contributions and feedback are welcome!
