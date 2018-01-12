#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import tf
import cv2
import yaml
import math

STATE_COUNT_THRESHOLD = 3

VALID_DISTANCE_STOP_LINE = 150
EARLY_STOP_STEPS = 3




class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        self.pose = None
        self.waypoints = None
        self.camera_image = None
        self.lights = []
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()

        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        self.publish_ground_truth = False
        self.has_image = False
        self.enable_early_stop = True

        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        self.loop()
        # rospy.spin()

    def loop(self):
        rate = rospy.Rate(10) # 10Hz
        while not rospy.is_shutdown():
            if not self.has_image and not self.publish_ground_truth:
                rospy.loginfo("TL_DETECTOR: No images received")
                continue
            if not self.lights:
                rospy.loginfo("TL_DETECTOR: No traffic lights received")
                continue

            '''
            Publish upcoming red lights at desired frequency.
            Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
            of times till we start using it. Otherwise the previous stable state is
            used.
            '''
            light_wp, state = self.process_traffic_lights()
            if self.state != state:
                self.state_count = 0
                self.state = state
            elif self.state_count >= STATE_COUNT_THRESHOLD:
                self.last_state = self.state
                light_wp = light_wp if state == TrafficLight.RED else -1
                self.last_wp = light_wp
                self.upcoming_red_light_pub.publish(Int32(light_wp))
            else:
                self.upcoming_red_light_pub.publish(Int32(self.last_wp))
            self.state_count += 1
            rate.sleep()

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.waypoints = waypoints.waypoints
        rospy.loginfo("TL_DETECTOR: Got waypoints")

    def traffic_cb(self, msg):
        self.lights = msg.lights

    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.has_image = True
        self.camera_image = msg
        # light_wp, state = self.process_traffic_lights()

    def distance(self, a, b):
        return math.sqrt((a.x - b.x)**2 + (a.y - b.y)**2 + (a.z - b.z)**2)

    def get_closest_waypoint(self, pose, waypoints):
        """Identifies the closest path waypoint to the given position
            https://en.wikipedia.org/wiki/Closest_pair_of_points_problem
        Args:
            pose (Pose): position to match a waypoint to

        Returns:
            int: index of the closest waypoint in self.waypoints
            double: distance of closest waypoint in self.waypoints
        """
        if waypoints is None:
            rospy.logwarn("TL_DETECTOR: no waypoints available")
            return -1

        closest_dist = 1000000.  # large umber
        closest_idx = 0
        for i, w in enumerate(waypoints):
            dist = self.distance(pose.position, w.pose.pose.position)
            if dist < closest_dist:
                closest_dist = dist
                closest_idx = i
        return closest_idx, closest_dist

    def get_light_state(self, light):
        """Determines the current color of the traffic light

        Args:
            light (TrafficLight): light to classify

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if self.pose and self.waypoints:
            car_wp_id = self.get_closest_waypoint(self.pose.pose, self.waypoints)
            # use traffic light waypoints instead of all
            traffic_light_wp_id, traffic_light_dist = self.get_closest_waypoint(self.pose.pose, self.lights)
            if traffic_light_wp_id < 0 or traffic_light_wp_id >= len(self.lights):
                rospy.logwarn("TL_DETECTOR: Invalid traffic light idx %i", traffic_light_wp_id)
                return -1, TrafficLight.UNKNOWN
            traffic_light = self.lights[traffic_light_wp_id]
            # get closest stopline waypoint index
            stop_line_pose = Pose()
            # use the fact that stop line waypoint index correspond to traffic light waypoints
            stop_line_pose.position.x = stop_line_positions[traffic_light_wp_id][0]
            stop_line_pose.position.y = stop_line_positions[traffic_light_wp_id][1]
            # back to all waypoints
            stop_line_wp_id, stop_line_dist = self.get_closest_waypoint(stop_line_pose, self.waypoints)
            if stop_line_wp_id < 0:
                rospy.logwarn("TL_DETECTOR: Invalid stop line idx %i", stop_line_wp_id)
                return -1, TrafficLight.UNKNOWN
            if self.enable_early_stop and stop_line_wp_id > EARLY_STOP_STEPS:
                light = stop_line_wp_id - EARLY_STOP_STEPS
            else:
                light = stop_line_wp_id

            rospy.logdebug("TL_DETECTOR: Car   x: %.2f, y: %.2f",
                           self.pose.pose.position.x,
                           self.pose.pose.position.y)
            rospy.logdebug("TL_DETECTOR: Stop  x: %.2f, y: %.2f, id: %i",
                           stop_line_pose.position.x,
                           stop_line_pose.position.y,
                           stop_line_wp_id)
            rospy.logdebug("TL_DETECTOR: Light x: %.2f, y: %.2f, id: %i, state: %i",
                           traffic_light.pose.pose.position.x,
                           traffic_light.pose.pose.position.y,
                           traffic_light_wp_id, traffic_light.state)

        if light:
            # Use ground truth of the simulator to determine light state
            if self.publish_ground_truth:
                return light, traffic_light.state
            # Use classifier to determine light state
            elif traffic_light_dist < VALID_DISTANCE_STOP_LINE:
                state = self.get_light_state(light)
                if state not in range(0,3):
                    rospy.logdebug("TL_DETECTOR: Unknown State %i from classifier", state)
                    return -1, TrafficLight.UNKNOWN
                return light, state

        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')

