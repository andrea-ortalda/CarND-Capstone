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
from scipy.spatial import KDTree

STATE_COUNT_THRESHOLD = 3
MAX_STOP_LINE_DIST = 150 # Max stop line distance [#waypoints]

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        # Parameters
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Subscribers
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        self.traffic_lights_sub = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        self.image_color_sub = rospy.Subscriber('/image_color', Image, self.image_cb) #TODO: evaluate using image_raw instead

        # Publishers
        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # TL detector node variables
        self.current_pose = None
        self.base_waypoints = None
        self.base_waypoints_init = False
        self.base_waypoints_tree = None
        self.camera_image = None
        self.current_traffic_lights = []
        self.bridge = CvBridge()
        self.light_classifier = TLClassifier()
        self.listener = tf.TransformListener()
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0

        self.step()

    def step(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            rate.sleep()

    '''
    This method updates the current ego vehicle pose
    '''
    def pose_cb(self, data):
        self.current_pose = data

    '''
    This method stores the base waypoints in the class variables
    '''
    def base_waypoints_cb(self, input_waypoints):
        if not self.base_waypoints_init:
            self.base_waypoints = input_waypoints
            self.base_waypoints_2d = [[wp.pose.pose.position.x, wp.pose.pose.position.y] for wp in input_waypoints.waypoints]
            self.base_waypoints_tree = KDTree(self.base_waypoints_2d)
            self.base_waypoints_init = True
            #rospy.loginfo('Base waypoints initialized.')

    '''
    This method stores the traffic light data in the class variables
    '''
    def traffic_cb(self, data):
        self.current_traffic_lights = data.lights

    '''
    This method identifies the current state of the traffic light in the incoming front camera sensor image
    '''
    def image_cb(self, data):
        self.has_image = True
        self.camera_image = data
        light_wp, state = self.process_traffic_lights()

        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            red = state == TrafficLight.RED or state == TrafficLight.UNKNOWN
            light_wp = light_wp if red else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    '''
    This method identifies the closest path waypoint to a given position
    '''
    def get_closest_waypoint(self, pose_x, pose_y):
        closest_wp_idx = self.base_waypoints_tree.query([pose_x,pose_y],1)[1]
        return closest_wp_idx

    '''
    This method determines the current state of the traffic light
    '''
    def get_light_state(self, light):
        if(not self.has_image):
            self.prev_light_loc = None
            return False

        cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

        #Get classification
        return self.light_classifier.get_classification(cv_image)

    '''
    This method finds the closest traffic light within a max distance and determines the distance to its stop line and its state
    '''
    def process_traffic_lights(self):
        closest_line_wp_idx = None
        closest_light = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.current_pose):
            current_pose_wp_idx = self.get_closest_waypoint(self.current_pose.pose.position.x, self.current_pose.pose.position.y)

            diff = len(self.base_waypoints.waypoints)
            for i, current_light in enumerate(self.current_traffic_lights):
                line = stop_line_positions[i]
                current_line_wp_idx = self.get_closest_waypoint(line[0], line[1])
                d = current_line_wp_idx - current_pose_wp_idx
                if d>=0 and d<diff and d<MAX_STOP_LINE_DIST:
                    diff = d
                    closest_light = current_light
                    closest_line_wp_idx = current_line_wp_idx

        if closest_light:
            #rospy.loginfo('Close traffic light found.')
            closest_light_state = self.get_light_state(closest_light)
            #closest_light_state = closest_light.state
            #rospy.loginfo('Current pose waypoint index: x = %s', current_pose_wp_idx)
            #rospy.loginfo('Closest stop line waypoint index: x = %s', closest_line_wp_idx)
            #rospy.loginfo('Closest traffic light state: %s', closest_light_state)
            if closest_light_state is None:
                closest_light_state = TrafficLight.UNKNOWN
            #if closest_light_state == TrafficLight.UNKNOWN:
                #rospy.loginfo('Unknown Traffic light state.')
            return closest_line_wp_idx, closest_light_state

        #rospy.loginfo('No close traffic light detected.')
        return -1, TrafficLight.UNKNOWN


if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
