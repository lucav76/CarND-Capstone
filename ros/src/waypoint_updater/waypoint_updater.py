#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

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
MAX_DECEL = 4. # Highest acceptable deceleration rate, in m/s^2
TARGET_DECEL = 1. # Ideal deceleration rate, in m/s^2
MAX_COMPLETE_STOP_DIST = 3. # If the traffic light is within this many meters, car can come to a complete stop

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers and publishers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Constants
        self.base_waypoints = None # taken from waypoint_loader; should never change once initialized
        self.mod_waypoints = None # base_waypoints with speeds modified for traffic lights
        self.pose = None # current position of the car
        self.tl_index = -1 # -1 if no traffic point detected; otherwise equal to the waypoint nearest the nearest red light
        self.change_tl_index = True # Will be set to true whenever tl_index changes; initialize to true to populate mod_waypoints

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.base_waypoints and self.pose:
                # Find the waypoint closest to the car
                # Get the position and yaw of the car in euler coordinates
                car_x = self.pose.position.x
                car_y = self.pose.position.y
                # Initialize closest point and its distance
                closest_point = None
                closest_distance = float('inf')
                # Loop through all the waypoints
                for i in range(0, len(self.base_waypoints)):
                    waypoint = self.base_waypoints[i]
                    dist = math.sqrt((car_x - waypoint.pose.pose.position.x) ** 2 + (car_y - waypoint.pose.pose.position.y) ** 2)
                    if dist < closest_distance:
                        closest_distance = dist
                        closest_point = i

                # TODO: For testing only: delete before deployment
                if closest_point == 380:
                    self.change_tl_index = True
                    self.tl_index = 417
                # TODO End TODO

                # Update waypoint velocities if tl_index has changed
                if self.change_tl_index:
                    # Whenever tl_index changes, calculate all waypoint speeds from the default waypoints
                    # This will populate mod_waypoints during the first iteration of the loop
                    # This will handle no red lights being detected because "-1" is always < any waypoint index
                    self.change_tl_index = False
                    self.mod_waypoints = self.base_waypoints

                    if self.tl_index > closest_point: # Do nothing if the tl_index behind the car
                        dist = self.distance(self.base_waypoints, closest_point, self.tl_index) # Distance from the car (assuming it's at the nearest waypoint) to the red light
                        car_vel = self.get_waypoint_velocity(self.base_waypoints[closest_point])
                        # Assume the car is currently going at the speed of the current waypoint
                        # Should be conservative
                        # Will not work if tl_index bounces around for a single traffic light
                        decel_rate = car_vel**2/(2*dist) # Deceleration required to stop by light, per equation vf^2 = vi^2 + 2ad, vf = 0

                        if decel_rate < MAX_DECEL: # Drive right through the light if it's too close
                            # Modify waypoint speeds to stop in time for waypoint
                            decel_rate = max(TARGET_DECEL, decel_rate) # Don't decelerate too slowly
                            rospy.loginfo("tl_index_update stopping for tl; decel_rate %s", decel_rate)
                            for i in range(closest_point, self.tl_index):
                                dist = self.distance(self.base_waypoints, i, self.tl_index)
                                vel_target = math.sqrt(2*decel_rate*dist)
                                vel_target = min(vel_target, car_vel) # Limit speed to that of nearest waypoint, which is generated by the '/waypoint_loader/velocity' parameter
                                if dist < MAX_COMPLETE_STOP_DIST:
                                    vel_target = 0
                                self.set_waypoint_velocity(self.mod_waypoints, i, vel_target)

                        else:
                            rospy.loginfo("tl_index_update cannot stop; decel required would be %s", decel_rate)

                # final_waypoints is the next LOOKAHEAD_WPS waypoints starting with the closest
                # (this assumes that the car should always travel through the waypoints in ascending order)
                final_waypoints = self.mod_waypoints[closest_point:closest_point + LOOKAHEAD_WPS]  # final_waypoints will get shorter as the last waypoint is approached

                # Publish
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = final_waypoints
                self.final_waypoints_pub.publish(lane)
                rospy.loginfo("waypoint_updater published final_waypoints; closest_point %s, closest_distance %s, len(final_waypoints) %s, tl_index %s, time %s", closest_point, closest_distance, len(final_waypoints), self.tl_index, lane.header.stamp)

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
        """If the waypoint of the next traffic light changes, update it and flag the main loop to update the waypoint speeds"""
        if msg != self.tl_index:
            self.tl_index = msg
            self.change_tl_index = True
            rospy.loginfo("tl_index_update tl_index changed to %s", self.tl_index)
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
