# ROS Basic Terminology  
**ROS = Robot Operating System**

This note summarizes the basic ROS terms in simple robotics language.

---

## 1. Node

A **node** is a small program or module that does **one specific job** inside a robot system.  
Nodes communicate with other nodes by sending and receiving data.

Examples:

```text
Camera node  → reads camera images
SLAM node    → builds a map
Control node → decides motor commands
Motor node   → sends commands to wheels
```

---

## 2. Publisher and Subscriber

A **publisher** is a node that sends data to a topic.

A **subscriber** is a node that receives or listens to data from a topic.

```text
Publisher → Topic → Subscriber
```


---

## 3. Topic

A **topic** is a named communication channel that nodes use to share data.

Common ROS topics:

```text
/cmd_vel        robot movement command
/odom           robot position estimate
/scan           LiDAR scan data
/camera/image   camera image
/tf             coordinate transforms
```

Example:

```text
Keyboard node publishes to /cmd_vel
Motor controller node subscribes to /cmd_vel
Robot moves based on the command
```

---

## 4. Package

A **package** is an organized folder for ROS code and files related to one robot function.

A package can contain:

```text
code
launch files
config files
URDF files
messages
scripts
```

Example package names:

```text
my_robot_navigation/
my_robot_slam/
my_robot_description/
```

Think of a package as a **project folder** for one part of the robot system.

---

## 5. Launch

A **launch file** starts multiple ROS nodes/settings with one command.

Instead of running many commands manually:

```bash
rosrun camera_node
rosrun slam_node
rosrun controller_node
```

you can use one launch file:

```bash
roslaunch my_robot bringup.launch
```


---

## 6. RViz

**RViz** is a visualization tool that shows what the robot “thinks” is happening.

RViz can display:

```text
robot model
LiDAR scans
camera images
map
robot position
planned path
coordinate frames
```

---

## 7. Rosbag

A **rosbag** records ROS topic data so you can replay it later.

During a real robot run, you can record topics such as:

```text
/camera/image
/scan
/odom
/cmd_vel
```

Example:

```bash
rosbag record /scan /odom /cmd_vel
```

Replay the recorded data later:

```bash
rosbag play example.bag
```

This is useful because you can test SLAM, navigation, or control code using the same recorded robot data multiple times.
---

## 8. URDF

**URDF** stands for **Unified Robot Description Format**.

It describes the robot’s physical structure, like a robot body blueprint.

It includes:

```text
links: robot parts
joints: how parts connect or move
sensors: camera, LiDAR, IMU, etc.
dimensions: size and shape
visuals: how the robot looks
```

---

## 9. Gazebo

**Gazebo** is a robot simulator.

Gazebo can simulate:

```text
robot movement
physics
gravity
collisions
sensors
world/environment
```
