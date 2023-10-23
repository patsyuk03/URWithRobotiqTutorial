# UR With Robotiq Hand-e Tutorial

This is a guide on how to create a custom Moveit configuration using Moveit Setup Assistant. Before starting the tutorial you should first install all of the necessary stuff from the Prerequisites section.

## Instructions:
* [Setup Connection with UR5e](#setup-connection-with-ur5e)
* [Create Custom Moveit Config](#create-custom-moveit-config)
* [Running A Sample Code](#running-a-sample-code)
* [Controlling Robotiq Gripper](#controlling-robotiq-gripper)

## Prerequisites
 * [Moveit](https://moveit.ros.org/install-moveit2/binary/)
 * [Universal Robots ROS2 Driver](https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver)

### Clone this workspace and build
```bash
mkdir colcon_ws/src
cd colcon_ws/src
git clone https://github.com/patsyuk03/URWithRobotiqTutorial.git
cd ..
colcon build
source install/setup.bash
```
This repository contains ROS2 driver and description for Robotiq Hand-e, additionally, it has a demo package that includes 1 demo that you can run with fake hardware and the demo for the real robot execution with Robotiq gripper and custom moveit config.

## **Setup Connection with UR5e** 
### 1. Setup network connection
<br><img src="img/network.png" alt="Network Connection" width="400"/>

### 2. Launch UR5e controller and moveit config

<br>**For work with fake hardware:**

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=xxx.xxx.x.xxx use_fake_hardware:=true initial_joint_controller:=joint_trajectory_controller launch_rviz:=false
```
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_fake_hardware:=true launch_rviz:=true
```
<br>**For work with real robot:**

```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=xxx.xxx.x.xxx use_fake_hardware:=false launch_rviz:=false initial_joint_controller:=scaled_joint_trajectory_controller
```
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_fake_hardware:=false launch_rviz:=true
```
### 3. If working with real robot start external_control program at teach pendant

## **Create Custom Moveit Config** 

In the previous step, we used the default moveit configuration which has only a robot with no gripper or table that it stands on. In this section, you will learn how to create a custom Moveit configuration using Moveit Setup Assistant.

### 1. Add table

Open file ur5e_with_robotiq_hande.xacro from the ur_robotiq_description package and add there the following code:

```bash
  <!-- Add table link -->
  <link name="table">
    <visual>
      <geometry>
        <box size="1.5 1.5 0.05"/>
      </geometry>
      <material name="white">
        <color rgba="1 1 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="1.5 1.5 0.05"/>
      </geometry>
    </collision>
  </link>

  <!-- Define table position relative to the world link -->
  <joint name="world_to_table" type="fixed">
    <parent link="world"/>
    <child link="table"/>
    <origin xyz="0 0 -0.15" rpy="0 0 0"/>
  </joint>
```

Build the workspace with added changes
```bash
cd colcon_ws
colcon build
```

### 2. Moveit Setup Assistant

```bash
ros2 run moveit_setup_assistant moveit_setup_assistant
```
First, select "Create New MoveIt Configuration Package" and browse the ur5e_with_robotiq_hande.xacro robot description from ur_robotiq_description package.
<br><img src="img/1.png" alt="MSA" width="400"/>
Then select "Load Files".
<br><img src="img/2.png" alt="MSA" width="400"/>
From the left bar choose "Self-Collisions" and press "Generate Collision Matrix". This step creates the collision matrix that lets the robot know of the possible collisions with itself, the gripper, or the table.
<br><img src="img/3.png" alt="MSA" width="400"/>
From the left bar select "Planning Groups" and press "Add Planning Group". Write the Group Name, select Kinematic Solver and Group Default Planner. Press "Add Kin. Chain". 
<br><img src="img/4.png" alt="MSA" width="400"/>
Choose the Base Link and the Tip Link from the robot links. In this case base_link and tool0.
<br><img src="img/5.png" alt="MSA" width="400"/>
The end result should look like this:
<br><img src="img/6.png" alt="MSA" width="400"/>
From the left bar select "Robot Poses" and click "Add Robot Pose". Choose the name for the pose and by dragging the trackbars create the desired pose.
<br><img src="img/7.png" alt="MSA" width="400"/>
Next, we need to add the controllers. Select "ROS2 Controllers" -> "Add Controller", and enter the name of the controller. Press "Add Joints Group".
<br><img src="img/9.png" alt="MSA" width="400"/>
Select "ur_manipulator" group and click "Save".
<br><img src="img/8.png" alt="MSA" width="400"/>
This should be the result:
<br><img src="img/10.png" alt="MSA" width="400"/>
Now do the same thing for "Moveit Controllers".
<br><img src="img/11.png" alt="MSA" width="400"/>
Add the author's information.
<br><img src="img/12.png" alt="MSA" width="400"/>
Finally, select "Configuration Files" from the left bar, click Browse, and select "URWithRobotiqTutorial" as the directory where to save the newly created MoveIt configuration package then add "/ur_robotiq_moveit_config". Press "Generate Package".
<br><img src="img/14.png" alt="MSA" width="400"/>

### 3. Modify generated package

1. Open file ur5e_with_robotiq_hande.urdf.xacro in ur_robotiq_moveit_config package and change initial_positions_file->initial_positions. The final content of the file should be the following:

<br><img src="img/modif1.png" alt="MSA" width="400"/>

2. Open file ros2_controllers.yaml in ur_robotiq_moveit_config package and delete 

```bash
joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

3. Save the files

### 3. See the result

```bash
cd colcon_ws
colcon build
source install/setup.bash 
ros2 launch ur_robotiq_moveit_config demo.launch.py
```
<br><img src="img/15.png" alt="MSA" width="400"/>

## **Running A Sample Code**
### 1. Launch PnP demo
<br>**For work with fake hardware:** 

```bash
ros2 launch ur_robotiq_demo ur_with_gripper_demo.launch.py use_fake_hardware:=true
```
```bash
ros2 run ur_robotiq_demo demo
```
<br>**For work with real robot:**

```bash
ros2 launch ur_robotiq_demo ur_with_gripper_demo.launch.py robot_ip:=xxx.xxx.x.xxx use_fake_hardware:=false
```
Start external_control program at teach pendant

```bash
ros2 run ur_robotiq_demo ur_with_gripper_demo
```

## **Controlling Robotiq Gripper**

### 1. Setup network connection
Same as in [Setup Connection with UR5e](#setup-connection-with-ur5e)

### 2. Launch gripper driver
```bash
ros2 launch robotiq_hande_ros2_driver gripper_bringup.launch.py robot_ip:=xxx.xxx.x.xxx
```
### 3. Run the test
```bash
ros2 run robotiq_hande_ros2_driver test 
```
