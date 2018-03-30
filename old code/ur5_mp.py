#!/usr/bin/env python

"""
    moveit_cartesian_path.py - Version 0.1 2016-07-28

    Based on the R. Patrick Goebel's moveit_cartesian_demo.py demo code.

    Plan and execute a Cartesian path for the end-effector.

    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2014 Patrick Goebel.  All rights reserved.
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.5

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:

    http://www.gnu.org/licenses/gpl.html
"""

import rospy, sys, numpy as np
import moveit_commander
from copy import deepcopy
from geometry_msgs.msg import Twist
import moveit_msgs.msg
from sensor_msgs.msg import Image
from ur5_notebook.msg import Tracker
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint


tracker = Tracker()



class ur5_mp:
    def __init__(self):
        rospy.init_node("ur5_mp", anonymous=False)
        self.cxy_sub = rospy.Subscriber('cxy', Tracker, self.tracking_callback, queue_size=1)
        self.cxy_pub = rospy.Publisher('cxy1', Tracker, queue_size=1)
        self.phase = 1

        self.track_flag = False
        self.default_pose_flag = True
        self.cx = 400.0
        self.cy = 400.0
        self.points=[]
        self.state_change_time = rospy.Time.now()

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        self.end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)
        self.arm.set_planning_time(0.1)
        self.arm.set_max_acceleration_scaling_factor(.2)
        self.arm.set_max_velocity_scaling_factor(.4)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(self.end_effector_link).pose

        # Initialize the waypoints list
        self.waypoints= []
        self.pointx = []
        self.pointy = []
        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters

        wpose.position.x = -0.2
        wpose.position.y = -0.2
        wpose.position.z = 0.3
        self.waypoints.append(deepcopy(wpose))

        wpose.position.x = 0.1052
        wpose.position.y = -0.4271
        wpose.position.z = 0.4005

        wpose.orientation.x = 0.4811
        wpose.orientation.y = 0.5070
        wpose.orientation.z = -0.5047
        wpose.orientation.w = 0.5000

        self.waypoints.append(deepcopy(wpose))


        if np.sqrt((wpose.position.x-start_pose.position.x)**2+(wpose.position.x-start_pose.position.x)**2 \
            +(wpose.position.x-start_pose.position.x)**2)<0.1:
            rospy.loginfo("Warnig: target position overlaps with the initial position!")

        # self.arm.set_pose_target(wpose)

        # Set the internal state to the current state
        self.arm.set_start_state_to_current_state()

        # Plan the Cartesian path connecting the waypoints

        """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
                self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

           Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
           poses specified as waypoints. Configurations are computed for every eef_step meters;
           The jump_threshold specifies the maximum distance in configuration space between consecutive points
           in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
           the actual RobotTrajectory.

        """
        plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


        # plan = self.arm.plan()

        # If we have a complete plan, execute the trajectory
        if 1-fraction < 0.2:
            rospy.loginfo("Path computed successfully. Moving the arm.")
            num_pts = len(plan.joint_trajectory.points)
            rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
            self.arm.execute(plan)
            rospy.loginfo("Path execution complete.")
        else:
            rospy.loginfo("Path planning failed")



    def cleanup(self):
        rospy.loginfo("Stopping the robot")

        # Stop any current arm movement
        self.arm.stop()

        #Shut down MoveIt! cleanly
        rospy.loginfo("Shutting down Moveit!")
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

    def tracking_callback(self, msg):

        self.track_flag = msg.flag1
        self.cx = msg.x
        self.cy = msg.y
        self.error_x = msg.error_x
        self.error_y = msg.error_y
        if len(self.pointx)>8:
            self.track_flag = True
        if self.phase == 2:
            self.track_flag = False
            self.phase = 1

        if (self.track_flag and -0.4 < self.waypoints[0].position.x and self.waypoints[0].position.x < 0.6):
            self.execute()
            self.default_pose_flag = False
        else:
            if not self.default_pose_flag:
                self.track_flag = False
                self.execute()
                self.default_pose_flag = True



    def execute(self):
        if self.track_flag:


            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []

            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # wpose.position.x = -0.5215
            # wpose.position.y = 0.2014
            # wpose.position.z = 0.4102


            if len(self.pointx)>8:
                if len(self.pointx)==9:
                    x_speed = np.mean(np.asarray(self.pointx[4:8]) - np.asarray(self.pointx[3:7]))
                    wpose.position.x += 2 * x_speed
                    wpose.position.z = 0.05


                else:
                    if len(self.pointx)==11:
                        tracker.flag2 = 1
                        self.cxy_pub.publish(tracker)

                    if len(self.pointx)<12:
                        x_speed = np.mean(np.asarray(self.pointx[4:8])-np.asarray(self.pointx[3:7]))
                        wpose.position.x += (x_speed-self.error_x*0.015/105)

                    else:
                        drop_pose = deepcopy(start_pose)
                        drop_pose.position.x = -0.200
                        drop_pose.position.y = 0.2000
                        drop_pose.position.z = 0.3000
                        drop_pose.orientation.x = 0.4811
                        drop_pose.orientation.y = 0.5070
                        drop_pose.orientation.z = -0.5047
                        drop_pose.orientation.w = 0.5000
                        # seq_y = np.arange(start_pose.position.y,drop_pose.position.y+0.03, 0.03)
                        # dx = np.linspace(start_pose.position.x,drop_pose.position.x, len(seq_y))
                        # dz = np.linspace(start_pose.position.z,drop_pose.position.z, len(seq_y))
                        # for _ in range(len(seq_y)):
                        #     wpose.position.x += dx[1]-dx[0]
                        #     wpose.position.y += 0.03
                        #     wpose.position.z += dz[1]-dz[0]

                        #self.waypoints.append(deepcopy(drop_pose))
                        print drop_pose
                        self.arm.set_pose_target(drop_pose)
                        self.arm.set_start_state_to_current_state()
                        #plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)
                        plan = self.arm.plan()

                        #if 1-fraction < 0.2:
                        self.track_flag = False
                        #rospy.loginfo("Path computed successfully. Moving the arm.")
                        #num_pts = len(plan.joint_trajectory.points)
                        self.arm.execute(plan)
                        rospy.loginfo("Path execution complete-phase2.")
                        self.phase = 2
                        tracker.flag2 = 0
                        self.cxy_pub.publish(tracker)


                        #


            # Set the next waypoint to the right 0.5 meters
            else:
                wpose.position.x -= self.error_x*0.05/105
                wpose.position.y += self.error_y*0.04/105
                wpose.position.z = 0.15
                #wpose.position.z = 0.4005

            if self.phase == 1:
                self.waypoints.append(deepcopy(wpose))


                self.pointx.append(wpose.position.x)
                self.pointy.append(wpose.position.y)

                # Set the internal state to the current state
                # self.arm.set_pose_target(wpose)

                self.arm.set_start_state_to_current_state()

                # Plan the Cartesian path connecting the waypoints

                """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
                        self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

                   Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
                   poses specified as waypoints. Configurations are computed for every eef_step meters;
                   The jump_threshold specifies the maximum distance in configuration space between consecutive points
                   in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
                   the actual RobotTrajectory.

                """
                plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


                # plan = self.arm.plan()

                # If we have a complete plan, execute the trajectory
                if 1-fraction < 0.2:
                    rospy.loginfo("Path computed successfully. Moving the arm.")
                    num_pts = len(plan.joint_trajectory.points)
                    rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                    self.arm.execute(plan)
                    rospy.loginfo("Path execution complete.")
                else:
                    rospy.loginfo("Path planning failed")

        else:
            # Get the current pose so we can add it as a waypoint
            start_pose = self.arm.get_current_pose(self.end_effector_link).pose

            # Initialize the waypoints list
            self.waypoints= []
            self.pointx = []
            self.pointy = []
            # Set the first waypoint to be the starting pose
            # Append the pose to the waypoints list
            wpose = deepcopy(start_pose)

            # Set the next waypoint to the right 0.5 meters

            wpose.position.x = 0.1052
            wpose.position.y = -0.4271
            wpose.position.z = 0.4005

            wpose.orientation.x = 0.4811
            wpose.orientation.y = 0.4994
            wpose.orientation.z = -0.5121
            wpose.orientation.w = 0.5069

            self.pointx.append(wpose.position.x)
            self.pointy.append(wpose.position.y)
            self.waypoints.append(deepcopy(wpose))
            # Set the internal state to the current state
            # self.arm.set_pose_target(wpose)

            self.arm.set_start_state_to_current_state()

            # Plan the Cartesian path connecting the waypoints

            """moveit_commander.move_group.MoveGroupCommander.compute_cartesian_path(
                    self, waypoints, eef_step, jump_threshold, avoid_collisios= True)

               Compute a sequence of waypoints that make the end-effector move in straight line segments that follow the
               poses specified as waypoints. Configurations are computed for every eef_step meters;
               The jump_threshold specifies the maximum distance in configuration space between consecutive points
               in the resultingpath. The return value is a tuple: a fraction of how much of the path was followed,
               the actual RobotTrajectory.

            """
            plan, fraction = self.arm.compute_cartesian_path(self.waypoints, 0.01, 0.0, True)


            # plan = self.arm.plan()

            # If we have a complete plan, execute the trajectory
            if 1-fraction < 0.2:
                rospy.loginfo("Path computed successfully. Moving the arm.")
                num_pts = len(plan.joint_trajectory.points)
                rospy.loginfo("\n# intermediate waypoints = "+str(num_pts))
                self.arm.execute(plan)
                rospy.loginfo("Path execution complete.")
            else:
                rospy.loginfo("Path planning failed")
        # print self.points




mp=ur5_mp()

rospy.spin()
