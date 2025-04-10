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
![Robot Walking](./media/move1.gif)

### Publish Initial Robot Mode
Set the robot mode to `STAND`:
```bash
ros2 topic pub /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand "{mode: 'STAND', robot_id: 1}" --once
```

### Verify Smoother Server and Teleop Twist Keyboard
Ensure the `smoother_server` and `/teleop_twist_keyboard` services are running. If not, check the logs or restart the launch file.

### Verify Teleop Node
Before controlling the robot, ensure the teleop node is running. Check the list of active nodes:
```bash
ros2 node list
```
Look for a node named `teleop_twist_keyboard`. If it is not running, start it:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```
### Controlling the Robot

#### Moving the Robot

The robot moves by publishing velocity commands to the `<robot_namespace>/cmd_vel` topic. By default, the robot is named robot1.

Example using `teleop_twist_keyboard`:

```bash
source install/local_setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```

#### Using PS5 Controller

To control the robot using a PS5 controller:

```bash
ros2 launch ps5_controller ps5_controller.launch.py
```

This allows you to use a PlayStation 5 controller to navigate the robot in the simulation.


![](./media/robot_move.gif)

Robot Modes

The robot supports several modes:

    REST – Default position in which the robot cannot move.
    STAND – Mode in which the robot can rotate in place.
    TROT – Walking mode.

The robot operates with 12 degrees of freedom. To enable rotation, switch the mode to "STAND" by publishing to the robot_mode topic.

Example (for a robot with namespace `robot1`):

```bash
ros2 topic pub /robot1/robot_mode quadropted_msgs/msg/RobotModeCommand "{mode: 'STAND', robot_id: 1}"
```

After switching modes, control the robot using velocity commands:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/robot1/cmd_vel
```


![](./media/move1.gif)

### Changing Robot Behavior

The robot can sit and stand using the `robot_behavior_command` service.

Example command:

```bash
ros2 service call /robot1/robot_behavior_command quadropted_msgs/srv/RobotBehaviorCommand "{command: 'walk'}"
```

Possible commands:

    walk – The robot stands up (REST) and can walk (TROT).
    up – The robot stands up (REST) and locks movement.
    sit – The robot sits down (STAND).

![](./media/sitUp.gif)

## Multi-Robot Setup and Model Switching

### Changing Robot Models

You can change between robot models (e.g., go2, go1) in gazebo_multi_nav2_world.launch.py file 102 str:

![](./media/switch.png)

for go2: use "go2_description" 
for go1: use "go1_description"

Running Multiple Robots Simultaneously
![](./media/go1multi.png)
![](./media/go2multi.png)
### The repository supports simultaneous operation of multiple robots. Each robot has access to nav2. In the robot.config file, add the robot’s namespace and spawn coordinates in the world.
![](./media/robot_config.png)

### NAV2 work demonstration: 
![](./media/robot-nav2.gif)


## Credits, thaks for all

    mike4192: (SpotMicro)[https://github.com/mike4192/spotMicro]
    Unitree Robotics: (A1 ROS)[https://github.com/unitreerobotics/a1_ros]
    QUADRUPED ROBOTICS: (Quadruped)[https://quadruped.de]
    lnotspotl: (GitHub)[https://github.com/lnotspotl]
    anujjain-dev: (Unitree-go2 ROS2)[https://github.com/anujjain-dev/unitree-go2-ros2]

## TODO

    Add Gazebo Classic support (physics and inertial parameters for URDF).
    Perform odometry calibration 

## Multi-Robot Setup and Model Switching

### Changing Robot Models
You can change between robot models (e.g., go2, go1) in `gazebo_multi_nav2_world.launch.py` file at line 102:

For go2: use `go2_description`
For go1: use `go1_description`

### Running Multiple Robots Simultaneously
The repository supports simultaneous operation of multiple robots. Each robot has access to nav2. In the `robot.config` file, add the robot’s namespace and spawn coordinates in the world.

## Credits

- mike4192: [SpotMicro](https://github.com/mike4192/spotMicro)
- Unitree Robotics: [A1 ROS](https://github.com/unitreerobotics/a1_ros)
- QUADRUPED ROBOTICS: [Quadruped](https://quadruped.de)
- lnotspotl: [GitHub](https://github.com/lnotspotl)
- anujjain-dev: [Unitree-go2 ROS2](https://github.com/anujjain-dev/unitree-go2-ros2)

## TODO

- Add Gazebo Classic support (physics and inertial parameters for URDF).
- Perform odometry calibration
