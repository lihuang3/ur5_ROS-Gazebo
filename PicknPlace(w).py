
# coding: utf-8

# In[6]:

import sys
import rospy
from moveit_commander import RobotCommander, PlanningSceneInterface, roscpp_initialize, roscpp_shutdown
from geometry_msgs.msg import PoseStamped
import numpy as np


# In[7]:

if __name__=='__main__':

    roscpp_initialize(sys.argv)
    rospy.init_node('moveit_py_demo', anonymous=True)

    scene = PlanningSceneInterface()
    robot = RobotCommander()
    rospy.sleep(1)

    # clean the scene
    robot.manipulator.detach_object("part")
    scene.remove_world_object("pole")
    scene.remove_world_object("table")
    scene.remove_world_object("part")
    
    # publish a demo scene
    p = PoseStamped()
    p.header.frame_id = robot.get_planning_frame()
    p.pose.position.x = 0.7
    p.pose.position.y = -0.4
    p.pose.position.z = 0.85
    p.pose.orientation.w = 1.0
    #p.pose.orientation.x = 2.0
    #p.pose.orientation.y = 3.0
    #p.pose.orientation.z = 2.0
    scene.add_box("pole", p, (0.3, 0.1, 1.0))

    p.pose.position.y = -0.2
    p.pose.position.z = 0.175
    scene.add_box("table", p,(0.15, 0.1, 0.3))

    p.pose.position.x = 0.6
    p.pose.position.y = -0.7
    p.pose.position.z = 0.5
    p.pose.orientation.w = 1.0
    #p.pose.orientation.x = 2.0
    #p.pose.orientation.y = 3.0
    #p.pose.orientation.z = 2.0
    scene.add_sphere("part", p, 0.15)

    rospy.sleep(1)

    robot.manipulator.attach_object("part", "wrist_3_joint")

    current = np.asarray(robot.manipulator.get_current_joint_values())
    current[0] += 0.2
    p = robot.manipulator.plan(current)

    rospy.spin()
    roscpp_shutdown()


# In[ ]:



