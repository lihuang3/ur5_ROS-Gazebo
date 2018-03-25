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
import geometry_msgs.msg
import moveit_msgs.msg
import cv2, cv_bridge
from sensor_msgs.msg import Image


from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

class MoveItCartesianPath:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/ur5/usbcam/image_raw', Image, self.image_callback)

        rospy.init_node("moveit_cartesian_path", anonymous=False)

        rospy.loginfo("Starting node moveit_cartesian_path")

        rospy.on_shutdown(self.cleanup)

        # Initialize the move_group API
        moveit_commander.roscpp_initialize(sys.argv)

        # Initialize the move group for the ur5_arm
        self.arm = moveit_commander.MoveGroupCommander('manipulator')

        # Get the name of the end-effector link
        end_effector_link = self.arm.get_end_effector_link()

        # Set the reference frame for pose targets
        reference_frame = "/base_link"

        # Set the ur5_arm reference frame accordingly
        self.arm.set_pose_reference_frame(reference_frame)

        # Allow replanning to increase the odds of a solution
        self.arm.allow_replanning(True)

        # Allow some leeway in position (meters) and orientation (radians)
        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.1)

        # Get the current pose so we can add it as a waypoint
        start_pose = self.arm.get_current_pose(end_effector_link).pose

        # Initialize the waypoints list
        waypoints = []

        # Set the first waypoint to be the starting pose
        # Append the pose to the waypoints list
        wpose = deepcopy(start_pose)

        # Set the next waypoint to the right 0.5 meters

        wpose.position.x = -0.2
        wpose.position.y = -0.2
        wpose.position.z = 0.3
        waypoints.append(deepcopy(wpose))

        wpose.position.x = 0.1052
        wpose.position.y = -0.4271
        wpose.position.z = 0.4005

        wpose.orientation.x = 0.4811
        wpose.orientation.y = 0.4994
        wpose.orientation.z = -0.5121
	wpose.orientation.w = 0.5069

        waypoints.append(deepcopy(wpose))
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
        plan, fraction = self.arm.compute_cartesian_path(waypoints, 0.01, 0.0, True)


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

    def image_callback(self,msg):
        # BEGIN BRIDGE
        image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
        # END BRIDGE
        # BEGIN HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # END HSV
        # BEGIN FILTER
        lower_red = np.array([ 0,  100, 100])
        upper_red = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower_red, upper_red)
        (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        #area = cv2.contourArea(cnts)
        h, w, d = image.shape
        # print h, w, d  (800,800,3)
        #BEGIN FINDER
        M = cv2.moments(mask)
        if M['m00'] > 0:
          cx = int(M['m10']/M['m00'])
          cy = int(M['m01']/M['m00'])

        # cx range (55,750) cy range( 55, ~ )
        # END FINDER
        # Isolate largest contour
        #  contour_sizes = [(cv2.contourArea(contour), contour) for contour in cnts]
        #  biggest_contour = max(contour_sizes, key=lambda x: x[0])[1]
          for i, c in enumerate(cnts):
              area = cv2.contourArea(c)
              if area > 7500:
                  cv2.circle(image, (cx, cy), 10, (0,0,0), -1)
                  cv2.putText(image, "({}, {})".format(int(cx), int(cy)), (int(cx-5), int(cy+15)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                  cv2.drawContours(image, cnts, -1, (255, 255, 255),1)
        # BEGIN circle

        #  print max(contour_sizes)[0]
          #area = cv2.contourArea(cnts)
          #print area
          #END circle

        cv2.namedWindow("window", 1)
        cv2.imshow("window", image )
        cv2.waitKey(5)

follower=MoveItCartesianPath()
rospy.spin()
