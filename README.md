#### 0. References
- [__`GitHub: utecrobotics/ur5`__](https://github.com/utecrobotics/ur5) testing ur5 motion 



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
To test UR5 motion, run [`testmotion.py`](testmotion.py)


#### 2. Moveit
[`Official tutorial`](http://docs.ros.org/kinetic/api/moveit_tutorials/html/)
Install Moveit 
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

#### 3 USB Camera Installation in ROS
[`Reference link`](https://answers.ros.org/question/197651/how-to-install-a-driver-like-usb_cam/)

To list all video devices picked up by the kernel
```angularjs
ls -ltrh /dev/video*
```


```angularjs
cd ur5_ws/src

git clone https://github.com/bosch-ros-pkg/usb_cam.git

cd ..

catkin_make

source devel/setup.bash

roscd usb_cam

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

#### 4. Using Gazebo Camera Plugins

- [`Tutorial: Using a URDF in Gazebo`](http://gazebosim.org/tutorials/?tut=ros_urdf#Tutorial:UsingaURDFinGazebo)
prerequisite for Gazebo plugins
    
- [`Tutorial: Using Gazebo plugins with ROS`](http://gazebosim.org/tutorials?tut=ros_gzplugins)
learning to use Gazebo plugins


- [`Viewer for ROS image topics`](http://wiki.ros.org/image_view)


- [`Camera implementation with IRIS drone`](http://discuss.px4.io/t/how-to-add-a-ros-camera-to-iris-for-gazebo-simulation/5118)
