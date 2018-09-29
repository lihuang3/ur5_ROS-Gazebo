### Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers. 
<p align="center">
<img src="https://github.com/lihuang3/ur5_notebook/blob/master/media/demo1.gif" width="400">
<img src="https://github.com/lihuang3/ur5_notebook/blob/master/media/demo2.gif" width="405">

This repository demonstrates UR5 pick-and-place in ROS and Gazebo. The UR5 uses a USB cam to detect a red box on a conveyor ([`ur5_vision.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_vision.py)), and publish its position. UR5 plans its motion ([`ur5_mp.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_mp.py)) to follow the box. Once the end-effector gets close enough to the box, it approaches the box with vacuum grippers turning on ([`ur5_gripper.py`](https://github.com/lihuang3/ur5_ROS-Gazebo/blob/master/ur5_gripper.py)). Since the vacuum gripper only provides limited force, we placed multiple grippers in order to lift the object. 

- Video demos:
  [`Simulation video`](https://youtu.be/Yj5DEocFa48)
  [`Hardware video`](https://youtu.be/FAYPbAhYoXw)

- Hardware implementation in UR3:
  [`ur3_ROS-hardware`](https://github.com/lihuang3/ur3_ROS-hardware.git)

- Update: based on feebacks from the community, we have made several key changes to this repository on 09/16/2018. Please update your code in case you have trouble reproducing the results.

- How to cite this repository: 
  ```
    Huang, L., Zhao, H., Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers, (2018), GitHub repository, https://github.com/lihuang3/ur5_ROS-Gazebo.git
  ```
  or BibTex
  ```
    @misc{Huang2018,
      author = {Huang, L., Zhao, H.},
      title = {Implementation of UR5 pick and place in ROS-Gazebo with a USB cam and vacuum grippers},
      year = {2018},
      publisher = {GitHub},
      journal = {GitHub repository},
      howpublished = {\url{https://github.com/lihuang3/ur5_ROS-Gazebo.git}}
    }
  ```

#### How to use this repository
- This project was tested in Ubuntu 16.04 with ROS kinetic.
- Make sure you have installed Python2.7 and some useful libraries/packages, such as Numpy, cv2, etc.
- Install ROS kinetic, Gazebo, universal robot, Moveit, RViz. 
- Assuming your universal robot workspace is named as `ur_ws`, download the repository to `ur_ws/src/`
  ```
  $ cd ur_ws/src
  $ git clone https://github.com/lihuang3/ur5_ROS-Gazebo.git
  ```
- Under `ur_ws/src`, there are two folders: one is the official `universal_robot`, and the other is `ur5_ROS-Gazebo`. Open file `ur5_joint_limited_robot.urdf.xacro` under `ur_ws/src/universal_robot/ur_description/urdf/`, and __make the following change to the joint limit:__
  ```
    shoulder_pan_lower_limit="${-2*pi}" shoulder_pan_upper_limit="${2*pi}"
  ```
- In the same directory, make a copy of `common.gazebo.xacro` and `ur5.urdf.xacro` in case of any malfunction. 
These two default files do not include camera and vacuum gripper modules. 
So we would replace these two files with customized files. 
Under directory `ur_ws/src/ur5_ROS-Gazebo/src/ur_description/`, copy `common.gazebo.xacro` and `ur5.urdf.xacro` to `ur_ws/src/universal_robot/ur_description/urdf/`.
- Build the code under directory `ur_ws/`,
  ```
  $ catkin_make
  $ source devel/setup.bash  
  ```
- Run the code with ROS and Gazebo
  ```
  $ roslaunch ur5_notebook initialize.launch 
  ```
- Things to work on: (1) vacuum grippers only provide limited force for lifting, so we had to use so many of them in order to pick up a light box. If you have any suggestions, please let us know. (2) UR5 motion planning is not in realtime, and hence you can ovserve a non-smooth motion of the end-effect in the camera view. 
 

#### 0. References
- [__`GitHub: utecrobotics/ur5`__](https://github.com/utecrobotics/ur5) testing ur5 motion
- [__`Very useful ROS blog`__](http://www.guyuehome.com/column/ros-explore) ROS探索
- [__`ROS下如何使用moveit驱动UR5机械臂`__](http://blog.csdn.net/jayandchuxu/article/details/54693870)
- [__`ROS UR industrial`__](http://wiki.ros.org/universal_robot/Tutorials/Getting%20Started%20with%20a%20Universal%20Robot%20and%20ROS-Industrial)
- [__`ROS modern driver`__](https://github.com/iron-ox/ur_modern_driver)
- [__`ur_hardware_interface.cpp`__](https://github.com/iron-ox/ur_modern_driver/blob/883070d0b6c0c32b78bb1ca7155b8f3a1ead416c/src/ur_hardware_interface.cpp) Replace this cpp
if there is hardware interface error during catkin make




__######## Warning! The rest of this README is still under construction ###########__




#### 1. Universal Robot 5 Installation
[`Official installation tutorial`](http://wiki.ros.org/universal_robot)

The following command lines are for ur5 installation in ros-kinetic
```angularjs
mkdir -p ur5_ws/src
cd ur5_ws/src

# retrieve the sources
git clone -b kinetic-devel https://github.com/ros-industrial/universal_robot.git

cd ~/ur5_ws

# checking dependencies
rosdep install --from-paths src --ignore-src --rosdistro kinetic

# buildin,
catkin_make

# if there is any error, try
# pip uninstall em
# pip install empy

# source this workspace (careful when also sourcing others)
cd ~/ur5_ws
source devel/setup.bash
```
To run UR5 in Gazebo and rviz (`source devel/setup.bash`)
```angularjs
roslaunch ur_gazebo ur5.launch limited:=true

roslaunch ur5_moveit_config ur5_moveit_planning_execution.launch sim:=true limited:=true

roslaunch ur5_moveit_config moveit_rviz.launch config:=true
```
To test UR5 motion, run [`testmotion.py`](src/testmotion.py)
Look into this link for [`straight line motion`](http://answers.gazebosim.org/question/15402/ur5-straight-line-motion/)

#### 2. Moveit
[`Official tutorial`](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)
##### 2.0 Install Moveit
```
sudo apt-get install ros-kinetic-moveit
```
launch teh Moveit Setup Assistant
```
roslaunch moveit_setup_assistant setup_assistant.launch

1. Click on the "Create New MoveIt Configuration Package" button,click the "Browse" button, select the xacro file you created in the Previous Chapter, and click on the "Load Files" button.
PATH: /src/universal_robot/ur_description/urdf/ur5.urdf.xacro

2. Go to the "Self-Collisions" tab, and click on the "Regenerate Collision Matrix" button.

3. move to the "Virtual Joints" tab. Here, you will define a virtual joint for the base of the robot. Click the "Add Virtual Joint" button, and set the name of this joint to FixedBase, and the parent to world.

4. open the "Planning Groups" tab and click the "Add Group" button. Now, you will create a new group called manipulator, which uses the KDLKinematicsPlugin.

3. Move Group Python InterFace Tutorial[`Official tutorial`](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html)

```
##### 2.1 Use Moveit Interface
[`Move Group Interface Tutorial`](http://docs.ros.org/indigo/api/moveit_tutorials/html/doc/pr2_tutorials/planning/src/doc/move_group_interface_tutorial.html#cartesian-paths)

##### 2.2 Use Moveit in Python
[`Cartesian Path Planning`](src/motionplanner2.py)

Using the ur5 with the MoveIt Motion Planning Framework for quick motion planning.
Install the package from package management, and run the MoveIt! planning demo:

```angularjs
$ sudo apt-get install ros-kinetic-ur5-moveit-config

$ roslaunch ur5_moveit_config demo.launch
```
Our goal is to move the universal robot (ur5) end effector moving in straight line (Cartesian path) with Moveit-Python interface.

<p align="center">
<img src="https://github.com/lihuang3/ur5_notebook/blob/master/media/CartesianPathCamera.gif" width="800">
</p>


__References__ \
[1] [CMobley7 commented on ros-planning/moveit_commander](https://github.com/ros-planning/moveit_commander/issues/51)\
[2] [homesick-nick UR5CubicInterpolation](https://github.com/nick-pestell/UR5CubicInterpolation/blob/master/cubic_interpolation.py)\
[3] [Move Group Python Interface Tutorial
](http://docs.ros.org/kinetic/api/moveit_tutorials/html/doc/pr2_tutorials/planning/scripts/doc/move_group_python_interface_tutorial.html) \
[4] [ur_modern_driver](https://github.com/ThomasTimm/ur_modern_driver)

#### 3 USB Camera Installation in ROS
[`Reference link`](https://answers.ros.org/question/197651/how-to-install-a-driver-like-usb_cam/)

To list all video devices picked up by the kernel
```angularjs
$ ls -ltrh /dev/video*
```


```angularjs
$ cd ur5_ws/src

$ git clone https://github.com/bosch-ros-pkg/usb_cam.git

$ cd ..

$ catkin_make

$ source devel/setup.bash

$ roscd usb_cam

# run `roscore` in a new terminal
# Make sure a usb cam is connected

```

To connect external cam.
Locate the usb_cam-test.launch file in folder

`cd ~/ur5_ws/src/usb_cam/launch`

Change

`<param name="video_device" value="/dev/video0" />`

to

`<param name="video_device" value="/dev/video1" />`

From

`cd ~/catkin-ws/src/usb_cam/launch`
run

`roslaunch usb_cam-test.launch`

If this works, quit the test program, open rviz
```angularjs
rosrun rviz rviz
```
run the following command in ur5_ws folder (`source devel/setup.bash`)
```angularjs
rosrun usb_cam usb_cam_node
```


#### 4. Revolute-Revolute Manipulator Robot
"[__`RRBot`__](https://github.com/ros-simulation/gazebo_ros_demos), or ''Revolute-Revolute Manipulator Robot'',
is a simple 3-linkage, 2-joint arm that we will use to demonstrate various
features of Gazebo and URDFs.
It essentially a double inverted pendulum and demonstrates some fun
control concepts within a simulator."

To get RRBot, clone the gazebo_ros_demos Github repo into the `/src` folder of your catkin workspace and rebuild your workspace:
```angularjs
cd ~/catkin_ws/src/
git clone https://github.com/ros-simulation/gazebo_ros_demos.git
cd ..
catkin_make
source devel/setup.bash
```

__Quick start__

Rviz:
```angularjs
roslaunch rrbot_description rrbot_rviz.launch
```

Gazebo:
```angularjs
roslaunch rrbot_gazebo rrbot_world.launch
```

[ROS Control](http://gazebosim.org/tutorials?tut=ros_control):
```angularjs
roslaunch rrbot_control rrbot_control.launch
```
Example of Moving Joints:
```angularjs
rostopic pub /rrbot/joint2_position_controller/command std_msgs/Float64 "data: -0.9"
```


#### 5. Using Gazebo Camera Plugins

- [`Tutorial: Using a URDF in Gazebo`](http://gazebosim.org/tutorials/?tut=ros_urdf#Tutorial:UsingaURDFinGazebo)
prerequisite for Gazebo plugins

- [`Tutorial: Using Gazebo plugins with ROS`](http://gazebosim.org/tutorials?tut=ros_gzplugins)
learning to use Gazebo plugins


- [`Viewer for ROS image topics`](http://wiki.ros.org/image_view)


- [`Camera implementation with IRIS drone`](http://discuss.px4.io/t/how-to-add-a-ros-camera-to-iris-for-gazebo-simulation/5118)


- [`ur_description wiki`](http://wiki.ros.org/ur_description)
To view and manipulate the arm models in rviz, install the package from package management and launch the following:
```angularjs
roslaunch ur_description ur5_upload.launch
roslaunch ur_description test.launch
```
You probably need to install urdf_tutorial:
```angularjs
cd ~/catkin_ws/src/
git clone https://github.com/ros/urdf_tutorial
cd ..
catkin_make
source devel/setup.bash
```

In rviz, the first task	is to choose the frame of reference	for	the	visualization. In left panel, __`Global Options/Fixed Frame`__,
choose a proper frame (e. g. __`world`__)

Next, we want to view the 3D model of the robot. To accomplish this, we will insert an instance of the `robot model` plugin
To add the robot model to the rviz scene, click the “Add” button and choose __`RobotModel`__

To test UR5 USB cam, run [`testvision.py`](src/testvision.py)

<p align="center">
<img src="https://github.com/lihuang3/ur5_notebook/blob/master/media/JointSpaceMotionCamera.gif" width="800">
</p>

#### 5. gazebo world Launch

- [`Defining Joints in Solidworks to URDF Exporter for a Conveyor Belt`](https://answers.ros.org/question/52309/defining-joints-in-solidworks-to-urdf-exporter-for-a-conveyor-belt/)


- [`Tutorial: simulator in gazebo`](http://wiki.ros.org/simulator_gazebo/Tutorials/Gazebo_ROS_API)

launch gazebo files :
```angularjs
roslaunch ur5_notebook initialize.launch
```
change object pose and twist with command line:
```angularjs
rosservice call /gazebo/set_model_state '{model_state: { model_name: red_box, pose: { position: { x: 0, y: 0 ,z: 1 }, orientation: {x: 0, y: 0, z: 0, w: 0 } }, twist: { linear: {x: 0.1 , y: 0 ,z: 0 } , angular: { x: 0.0 , y: 0 , z: 0.0 } } , reference_frame: world } }'
```

#### 6. openCV
- [`Detect red circles in an image using OpenCV
`](https://solarianprogrammer.com/2015/05/08/detect-red-circles-image-using-opencv/)

- [`OpenCV moment
`](https://docs.opencv.org/3.1.0/dd/d49/tutorial_py_contour_features.html)

- [`OpenCV moment
`](https://docs.opencv.org/3.4.0/dd/d49/tutorial_py_contour_features.html)

- [`cv2.contourArea() examples
`](https://www.programcreek.com/python/example/86843/cv2.contourArea)

##### 7. ariac
- [`ARIAC 2017: INSTALL 4.2 4.3
`](http://wiki.ros.org/ariac/Tutorials/SystemSetup)
