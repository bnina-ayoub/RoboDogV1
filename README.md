# Unitree go2, go1 simulation in Gazebo Sim

This repository allows you to run dog robots in the GAZEBO simulator. The robot can walk, rotate with 12 degrees of freedom, and features a `robot_msgs` interface. The robot moves using inverse kinematics, and its odometry is based on direct kinematics. Additionally, all functionalities are developed in Python.

> **Note:** Before launching, ensure that you install all dependencies and build the project using `colcon build`.

---

## Setup and Installation

### Clone the Repository and Build

```bash
mkdir -p ~/go_sim/src
cd ~/go_sim/src
git clone https://github.com/abutalipovvv/go_sim_py.git .
cd ..
colcon build --symlink-install
```

### Install Dependencies

```bash
cd ~/go_sim
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

## Environment Configuration

### Export Gazebo Models Path

Before running the simulation, export the path to your Gazebo models:

```bash
export GZ_SIM_RESOURCE_PATH=/home/$USER/go_sim/gazebo_sim/models
```
(Replace with the correct path to your models.)

### Configure CycloneDDS

To support multiple topics, configure CycloneDDS by creating a configuration file (e.g., cyclonedds.xml) with the following content:

```xml
<CycloneDDS>
  <Domain>
    <General>
      <Interfaces>
        <NetworkInterface name="lo" multicast="true" />
      </Interfaces>
      <DontRoute>true</DontRoute>
    </General>
    <Discovery>
      <ParticipantIndex>auto</ParticipantIndex>
      <MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>
    </Discovery>
  </Domain>
</CycloneDDS>
```
Then, set the environment variable to point to this file:

```bash
export CYCLONEDDS_URI=file:///home/$USER/go_sim/cyclonedds.xml
```

## Running the Simulation

### Source the Workspace
Before running any commands, source the workspace:
```bash
source install/setup.bash
```

### Launch the Simulation
To launch the Gazebo simulation with multiple robots:
```bash
ros2 launch gazebo_sim gazebo_multi_nav2_world.launch.py
```

### Publish Initial Robot Mode
Set the robot mode to `STAND`:
```bash
ros2 topic pub /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand "{mode: 'STAND', robot_id: 1}" --once
```

### Verify Smoother Server
Ensure the `smoother_server` service is running. If not, check the logs or restart the launch file.

### Controlling the Robot
Use teleop to control the robot:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

### Verify Teleop Node
Before controlling the robot, ensure the teleop node is running. Check the list of active nodes:
```bash
ros2 node list
```
Look for a node named `teleop_twist_keyboard`. If it is not running, start it:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

## Multi-Robot Setup and Model Switching

### Changing Robot Models
You can change between robot models (e.g., go2, go1) in `gazebo_multi_nav2_world.launch.py` file at line 102:

For go2: use `go2_description`
For go1: use `go1_description`

### Running Multiple Robots Simultaneously
The repository supports simultaneous operation of multiple robots. Each robot has access to nav2. In the `robot.config` file, add the robot’s namespace and spawn coordinates in the world.

## Preparing to Push to Forked Repository

### Add Remote Repository
Ensure your forked repository is added as a remote:
```bash
git remote add origin <your-forked-repo-url>
```

### Commit Changes
Stage and commit your changes:
```bash
git add .
git commit -m "Updated simulation setup and documentation"
```

### Push to Forked Repository
Push the changes to your forked repository:
```bash
git push origin main
```

## Credits

- mike4192: [SpotMicro](https://github.com/mike4192/spotMicro)
- Unitree Robotics: [A1 ROS](https://github.com/unitreerobotics/a1_ros)
- QUADRUPED ROBOTICS: [Quadruped](https://quadruped.de)
- lnotspotl: [GitHub](https://github.com/lnotspotl)
- anujjain-dev: [Unitree-go2 ROS2](https://github.com/anujjain-dev/unitree-go2-ros2)

## TODO

- Add Gazebo Classic support (physics and inertial parameters for URDF).
- Perform odometry calibration
