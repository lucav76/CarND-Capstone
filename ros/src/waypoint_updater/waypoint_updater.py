#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
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
MAX_DECEL = 10. # Highest acceptable deceleration rate, in m/s^2
TARGET_DECEL = 1.0 # Ideal deceleration rate, in m/s^2
EARLY_STOP_DIST = 4. # If the traffic light is within this many meters, car can come to a complete stop
EXTRA_STOP_WPS = 50

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers and publishers
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        rospy.Subscriber('/current_velocity', TwistStamped, self.current_velocity_cb, queue_size=1)
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Constants
        self.base_waypoints = None # taken from waypoint_loader
        self.pose = None # current position of the car
        self.tl_index = -1 # -1 if no traffic point detected; otherwise equal to the waypoint nearest the nearest red light
        self.change_tl_index = False # Will be set to true whenever tl_index changes
        self.std_velocity = self.kmph2mps(rospy.get_param('/waypoint_loader/velocity')) # Max speed parameter, in m/s
        self.max_waypoint_modified = 0 # The index of the furthest waypoint that has been modified, to speed up reseting base_waypoints after a light goes green
        self.current_velocity = None

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.base_waypoints and self.pose and self.current_velocity:
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

                # Update waypoint velocities if tl_index has changed
                if self.change_tl_index:
                    # Whenever tl_index changes, calculate all waypoint speeds
                    # This will handle no red lights being detected because "-1" is always < any waypoint index
                    self.change_tl_index = False

                    if self.tl_index > closest_point: # Do nothing if the tl_index behind the car
                        dist = self.distance(self.base_waypoints, closest_point, self.tl_index) - EARLY_STOP_DIST # Distance from the car (assuming it's at the nearest waypoint) to slightly ahead of the red light
                        dist = max(0.0001, dist) # Avoid dividing by zero or negative distance
                        decel_rate = self.current_velocity**2/(2*dist)

                        if decel_rate < MAX_DECEL: # Drive right through the light if it's too close
                            # Modify waypoint speeds to stop in time for waypoint
                            decel_rate = max(TARGET_DECEL, decel_rate) # Don't decelerate too slowly.  This will also make the car accelerate if initially stopped.
                            for i in range(closest_point, self.tl_index):
                                dist = self.distance(self.base_waypoints, i, self.tl_index) - EARLY_STOP_DIST
                                dist = max(0., dist) # Avoid negative distance
                                vel_target = math.sqrt(2*decel_rate*dist)
                                vel_target = min(vel_target, self.std_velocity) # Limit speed to '/waypoint_loader/velocity' parameter
                                self.set_waypoint_velocity(self.base_waypoints, i, vel_target)
                            for i in range(self.tl_index + 1, self.tl_index + EXTRA_STOP_WPS): # Set a few extra waypoints to zero speed
                                self.set_waypoint_velocity(self.base_waypoints, i, 0.)
                            self.max_waypoint_modified = self.tl_index + EXTRA_STOP_WPS
                            rospy.loginfo("tl_index_update stopping for tl; decel_rate %s", decel_rate)

                        else:
                            self.reset(closest_point)  # This should reset speeds
                            rospy.loginfo("tl_index_update cannot stop; decel required would be %s", decel_rate)
                    else:
                        self.reset(closest_point) # This should reset speeds
                        rospy.loginfo("tl_index_update new tl_index of %s behind car", self.tl_index)

                # final_waypoints is the next LOOKAHEAD_WPS waypoints starting with the closest
                # (this assumes that the car should always travel through the waypoints in ascending order)
                final_waypoints = self.base_waypoints[closest_point:closest_point + LOOKAHEAD_WPS]  # final_waypoints will get shorter as the last waypoint is approached

                # Publish
                lane = Lane()
                lane.header.frame_id = '/world'
                lane.header.stamp = rospy.Time.now()
                lane.waypoints = final_waypoints
                self.final_waypoints_pub.publish(lane)
                rospy.loginfo("waypoint_updater published final_waypoints; closest_point %s, closest_distance %s, car_vel %s, len(final_waypoints) %s, tl_index %s, time %s", closest_point, closest_distance, self.get_waypoint_velocity(self.base_waypoints[closest_point]), len(final_waypoints), self.tl_index, lane.header.stamp)

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
        tl_wp = msg.data
        if tl_wp != self.tl_index:
            self.tl_index = tl_wp
            self.change_tl_index = True
            rospy.loginfo("tl_index_update tl_index changed to %s", self.tl_index)
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def current_velocity_cb(self, msg):
        self.current_velocity = msg.twist.linear.x

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

    def reset(self, closest_wp):
        """Resets all modified waypoints to the std velocity.  May cause some issues if there is a traffic light near the end of the road."""
        if closest_wp < self.max_waypoint_modified:
            for i in range(closest_wp, self.max_waypoint_modified):
                self.set_waypoint_velocity(self.base_waypoints, i, self.std_velocity)
        pass


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')

