#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint

import math

import tf
import numpy as np

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
MAX_DECEL = 3. # Highest acceptable deceleration rate, in m/s^2
TARGET_DECEL = 1. # Ideal deceleration rate, in m/s^2
MAX_COMPLETE_STOP_DIST = 3. # If the traffic light is within this many meters, car can come to a complete stop

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.base_waypoints = None
        self.pose = None
        self.tl_index = 417 # TODO: Initialize to some big number, or none, once traffic_cb implemented
        self.std_velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity')) # Max speed parameter, in m/s
        self.change_tl_index = True # TODO: Initialize to false, and set to true in traffic_cb if tl_index gets changed

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.base_waypoints and self.pose:
                # Get the position and yaw of the car in euler coordinates
                car_x = self.pose.position.x
                car_y = self.pose.position.y

                # Loop through the waypoints to find the closest
                closest_point = None
                closest_distance = float('inf')
                for i in range(0, len(self.base_waypoints)):
                    waypoint = self.base_waypoints[i]
                    dist = math.sqrt((car_x - waypoint.pose.pose.position.x) ** 2 + (car_y - waypoint.pose.pose.position.y) ** 2)
                    if dist < closest_distance:
                        closest_distance = dist
                        closest_point = i

                # Update waypoint velocities if they need to change
                if self.change_tl_index:
                    self.change_tl_index = False
                    rospy.loginfo("waypoint_updater tl_index_update change noticed")

                    if self.tl_index > closest_point: # Do nothing if the tl_index behind the car
                        dist = self.distance(self.base_waypoints, closest_point, self.tl_index) # Distance from the car (assuming it's at the nearest waypoint) to the location of the red light
                        car_vel = self.get_waypoint_velocity(self.base_waypoints[closest_point]) # Assume the car is currently going at the speed of the current waypoint - should be conservative
                        decel_rate = car_vel**2/(2*dist) # Per equation vf^2 = vi^2 + 2ad; vf = 0

                        if decel_rate < MAX_DECEL: # Drive right through the light if it's too close
                            decel_rate = max(TARGET_DECEL, decel_rate) # Don't decelerate too slowly
                            rospy.loginfo("waypoint_updater tl_index_update stopping for tl; decel_rate %s", decel_rate)

                            for i in range(closest_point, self.tl_index):
                                # TODO Need logic for all traffic lights disappearing
                                dist = self.distance(self.base_waypoints, i, self.tl_index)
                                vel_target = math.sqrt(2*decel_rate*dist)
                                vel_target = min(vel_target, self.std_velocity)
                                if dist < MAX_COMPLETE_STOP_DIST:
                                    vel_target = 0
                                self.set_waypoint_velocity(self.base_waypoints, i, vel_target)

                # final_waypoints is the next LOOKAHEAD_WPS waypoints starting with the closest
                # (this assumes that the car should always travel through the waypoints in ascending order)
                final_waypoints = self.base_waypoints[closest_point:closest_point + LOOKAHEAD_WPS]  # final_waypoints will get shorter as the last waypoint is approached

                # Publish
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = final_waypoints
                self.final_waypoints_pub.publish(lane)
                rospy.loginfo("waypoint_updater published final_waypoints; closest_point %s, closest_distance %s, len(final_waypoints) %s, time %s", closest_point, closest_distance, len(final_waypoints), lane.header.stamp)

            rate.sleep()

    def pose_cb(self, msg):
        """Saves the position topic message without the top header."""
        self.pose = msg.pose
        # rospy.loginfo("waypoint_updater received a position")
        pass

    def waypoints_cb(self, waypoints):
        """Saves the waypoints topic message without the top header."""
        self.base_waypoints = waypoints.waypoints
        # rospy.loginfo("waypoint_updater base waypoints obtained")
        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
