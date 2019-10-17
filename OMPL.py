
# coding: utf-8

# In[1]:

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseStamped
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


# In[4]:

moveit_commander.roscpp_initialize(sys.argv)

robot = moveit_commander.RobotCommander()

scene = moveit_commander.PlanningSceneInterface()

group = moveit_commander.MoveGroupCommander("manipulator")

rospy.init_node("OMPL")

p = PoseStamped()
p.header.frame_id = robot.get_planning_frame()
p.pose.position.x = 0
p.pose.position.y = -0.6
p.pose.position.z = 0.01
scene.attach_box("base_link", "table_base", p, (1,0.5,0.02))

p.pose.position.x = 0.3
p.pose.position.y = -0.6
p.pose.position.z = 0.25
scene.attach_box("base_link", "left_wall", p, (0.02,0.5,0.5))

p.pose.position.x = -0.3
p.pose.position.y = -0.6
p.pose.position.z = 0.25
scene.attach_box("base_link", "right_wall", p, (0.02,0.5,0.5))

group_variable_values = group.get_current_joint_values()


current_state = robot.get_current_state()
joint_name = current_state.joint_state.name
joint_values = current_state.joint_state.position
joint_command = {name:value for name, value in zip(joint_name, joint_values) if name.startswith('manipulator')}
joint_state = robot.get_joint('wrist_3_joint')
joint_state.header = Header()

joint_state.header.stamp = rospy.Time.now()
joint_state.name = joint_command.keys()
joint_state.position = joint_command.values()
moveit_robot_state = RobotState()
moveit_robot_state.joint_state = joint_state
group.set_start_state(moveit_robot_state)

target_joint_values = target
group.set_joint_value_target(target_joint_values)
rospy.loginfo( 'About to plan')

plan = group.plan()
rospy.loginfo('Planning done')
rospy.sleep(3)

rospy.loginfo('Plan should execute now')
group.execute(plan)
rospy.loginfo('Plan executed')


# In[ ]:



