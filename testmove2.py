#!/usr/bin/env python
import time
import roslib; roslib.load_manifest('ur_driver')

import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from math import pi

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
import geometry_msgs.msg
import moveit_msgs.msg

from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


JOINT_NAMES = ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
               'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
Q1 = [2.2,0,-1.57,0,0,0]
Q2 = [1.5,0,-1.57,0,0,0]
Q3 = [1.5,-0.2,-1.57,0,0,0]
    
client = None

def move1():
    global joints_pos
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_disordered():
    order = [4, 2, 3, 1, 5, 0]
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = [JOINT_NAMES[i] for i in order]
    q1 = [Q1[i] for i in order]
    q2 = [Q2[i] for i in order]
    q3 = [Q3[i] for i in order]
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_repeated():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        d = 2.0
        g.trajectory.points = [JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0))]
        for i in range(10):
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 1
            g.trajectory.points.append(
                JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(d)))
            d += 2
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def move_interrupt():
    g = FollowJointTrajectoryGoal()
    g.trajectory = JointTrajectory()
    g.trajectory.joint_names = JOINT_NAMES
    try:
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]

        client.send_goal(g)
        time.sleep(3.0)
        print "Interrupting"
        joint_states = rospy.wait_for_message("joint_states", JointState)
        joints_pos = joint_states.position
        g.trajectory.points = [
            JointTrajectoryPoint(positions=joints_pos, velocities=[0]*6, time_from_start=rospy.Duration(0.0)),
            JointTrajectoryPoint(positions=Q1, velocities=[0]*6, time_from_start=rospy.Duration(2.0)),
            JointTrajectoryPoint(positions=Q2, velocities=[0]*6, time_from_start=rospy.Duration(3.0)),
            JointTrajectoryPoint(positions=Q3, velocities=[0]*6, time_from_start=rospy.Duration(4.0))]
        client.send_goal(g)
        client.wait_for_result()
    except KeyboardInterrupt:
        client.cancel_goal()
        raise
    except:
        raise

def cart():
    # Initialize the move_group API
    moveit_commander.roscpp_initialize(sys.argv)

    # Initialize the move group for the ur5_arm
    arm = moveit_commander.MoveGroupCommander('manipulator')

    # Get the name of the end-effector link
    end_effector_link = arm.get_end_effector_link()

    # Set the reference frame for pose targets
    reference_frame = "/base_link"

    # Set the ur5_arm reference frame accordingly
    arm.set_pose_reference_frame(reference_frame)

    # Allow replanning to increase the odds of a solution
    arm.allow_replanning(True)

    # Allow some leeway in position (meters) and orientation (radians)
    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.1)

    # Get the current pose so we can add it as a waypoint
    start_pose = arm.get_current_pose(end_effector_link).pose

    # Initialize the waypoints list
    waypoints = []

    # Set the first waypoint to be the starting pose
    # Append the pose to the waypoints list
    wpose = deepcopy(start_pose)


    # Set the next waypoint to the right 0.5 meters
    #

    wpose.position.x = 0.2377
    wpose.position.y = 0.1931
    wpose.position.z = 0.3391
    wpose.orientation.x = 0.7023
    wpose.orientation.y = 0.7086
    wpose.orientation.z = 0.0472
    wpose.orientation.w = 0.0481

    waypoints.append(deepcopy(wpose))

    wpose.position.x = 0.2935
    wpose.position.y = 0.1936
    wpose.position.z = 0.3324



    waypoints.append(deepcopy(wpose))

    wpose.position.x = 0.3076
    wpose.position.y = 0.1935
    wpose.position.z = 0.2283


    waypoints.append(deepcopy(wpose))


    wpose.position.x = 0.2216
    wpose.position.y = 0.1927
    wpose.position.z = 0.2166


    waypoints.append(deepcopy(wpose))

    # arm.set_pose_target(wpose)


    # Set the internal state to the current state


    # Plan the Cartesian path connecting the waypoints

    """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
            self, waypoints, eef_step, jump_threshold, avoid_collisios= True)
    
       Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
       poses specified as waypoints. Configurations are computed for every eef_step meters;
       The jump_threshold specifies the maximum distance in configuration space between consecutive points
       in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
       the actual RobotTrajectory.
    
    """

    for i in range(100):

        current_pose = arm.get_current_pose(end_effector_link).pose
        print ("Current pose: "),
        print current_pose

        arm.set_start_state_to_current_state()

        plan, fraction = arm.compute_cartesian_path([waypoints[i%4]], 0.01, 0.0, True)

        arm.execute(plan)
    # plan = arm.plan()



def main():
    global client



    try:
        rospy.init_node("test_move", anonymous=True, disable_signals=True)
        client = actionlib.SimpleActionClient('follow_joint_trajectory', FollowJointTrajectoryAction)
        print "Waiting for server..."
        client.wait_for_server()
        print "Connected to server"
        parameters = rospy.get_param(None)
        index = str(parameters).find('prefix')
        if (index > 0):
            prefix = str(parameters)[index+len("prefix': '"):(index+len("prefix': '")+str(parameters)[index+len("prefix': '"):-1].find("'"))]
            for i, name in enumerate(JOINT_NAMES):
                JOINT_NAMES[i] = prefix + name
        print "This program makes the robot move between the following three poses:"
        print str([Q1[i]*180./pi for i in xrange(0,6)])
        print str([Q2[i]*180./pi for i in xrange(0,6)])
        print str([Q3[i]*180./pi for i in xrange(0,6)])
        print "Please make sure that your robot can move freely between these poses before proceeding!"
        inp = raw_input("Continue? y/n: ")[0]
        if (inp == 'y'):
            #move1()
            cart()
            #move_repeated()
            #move_disordered()
            #move_interrupt()
        else:
            print "Halting program"
    except KeyboardInterrupt:
        rospy.signal_shutdown("KeyboardInterrupt")
        raise

if __name__ == '__main__': main()

