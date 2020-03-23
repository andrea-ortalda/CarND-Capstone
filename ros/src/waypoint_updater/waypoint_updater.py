#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
import numpy as np
from std_msgs.msg import Int32

import math

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
STOPLINE_DIST = 3 # Distance [#waypoints] to stop before stop line from center of the ego vehicle
MAX_STOP_DECEL = 1.0 # Max deceleration for stopping on a red traffic light [#waypoints / s^2]
MIN_VEL = 1.0 # Min velocity for moving [#waypoints / s]

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscribers
        self.current_pose_sub = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        self.base_waypoints_sub = rospy.Subscriber('/base_waypoints', Lane, self.base_waypoints_cb)
        self.traffic_light_sub = rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        # TODO (optional): Add a subscriber for /obstacle_waypoint

        # Publishers
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Current ego vehicle pose class variable
        self.current_pose = None

        # Base waypoints variables
        self.base_waypoints_init = False
        self.base_waypoints = None
        self.base_waypoints_2d = None
        self.base_waypoints_tree = None

        # Next stopline waypoint (with red traffic light) variable
        self.stopline_wp_idx = None

        self.step()

    '''
    This method updates the current ego vehicle pose
    '''
    def pose_cb(self, data):
        self.current_pose = data
        #rospy.loginfo('Current pose updated: x = %s , y = %s', self.current_pose.pose.position.x, self.current_pose.pose.position.y)

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
    The step method updates the final waypoints based on the current ego vehicle pose
    '''
    def step(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.current_pose and self.base_waypoints_init:
                self.publish_waypoints()
            rate.sleep()

    '''
    This method finds the closest waypoint to the current ego vehicle pose
    '''
    def get_closest_waypoint_idx(self):
        current_pose_x = self.current_pose.pose.position.x
        current_pose_y = self.current_pose.pose.position.y
        closest_idx = self.base_waypoints_tree.query([current_pose_x, current_pose_y], 1)[1]

        # Checking if closest waypoint is ahead or behind ego vehicle pose
        closest_wp = self.base_waypoints_2d[closest_idx]
        previous_wp = self.base_waypoints_2d[closest_idx-1]
        closest_wp_vec = np.array(closest_wp)
        previous_wp_vec = np.array(previous_wp)
        current_pose_vec = np.array([current_pose_x, current_pose_y])

        dot_product_result = np.dot(closest_wp_vec - previous_wp_vec, current_pose_vec - closest_wp_vec)

        # Choose next waypoint if the current one is behind ego vehicle pose
        if dot_product_result > 0:
            closest_idx = (closest_idx + 1)%len(self.base_waypoints_2d)

        #rospy.loginfo('Closest waypoint idx = %s', closest_idx)
        return closest_idx

    '''
    This method publishes the waypoints next to the ego vehicle pose
    '''
    def publish_waypoints(self):
        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    '''
    This method generates the next waypoints from base waypoints and current ego vehicle pose
    '''
    def generate_lane(self):
        lane = Lane()
        closest_wp_idx = self.get_closest_waypoint_idx()
        furthest_wp_idx = closest_wp_idx + LOOKAHEAD_WPS
        next_base_waypoints = self.base_waypoints.waypoints[closest_wp_idx:furthest_wp_idx]

        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= furthest_wp_idx or not self.stopline_wp_idx:
            lane.waypoints = next_base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(next_base_waypoints, closest_wp_idx)

        return lane

    '''
    This method updates the velocity of the next waypoints based on next stopline waypoint (with red traffic light)
    '''
    def decelerate_waypoints(self, waypoints, closest_wp_idx):
        decel_waypoints = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose
            stop_idx = max(self.stopline_wp_idx - closest_wp_idx - STOPLINE_DIST, 0)
            stop_dist = self.distance(waypoints, i, stop_idx)
            target_vel = math.sqrt(2 * MAX_STOP_DECEL * stop_dist)
            if target_vel < MIN_VEL:
                target_vel = 0.0
            p.twist.twist.linear.x = min(target_vel, wp.twist.twist.linear.x)
            decel_waypoints.append(p)
        return decel_waypoints

    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO (optional): Callback for /obstacle_waypoint message. We will implement it later
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


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
