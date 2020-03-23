
import glob
import shutil
import os
import itertools
import pandas as pd
import click
import rosbag
import rospy
from cv_bridge import CvBridge
import cv2

meta_files = ['Description.xml', 'RosParam.yaml', 'StaticTfMsgs.xml']

topics = ['/styx_msgs/TrafficLightArray']

tl_data_list = []

def save(input_bag_name):
    try:
        tl_data = pd.concat(tl_data_list, ignore_index=True).drop_duplicates()
        tl_data.to_csv('tl_data/'+input_bag_name+'labels.csv', index=False)
    except ValueError:
        pass

def cmd(input_bag_name):
    print('Input Bag', input_bag_name)
    input_bag = rosbag.Bag(input_bag_name)

    msg_count = 0
    for topic, msg, time_of_message in input_bag:
        if topic=="/vehicle/traffic_lights" and msg.lights:
            for light in msg.lights:
                msg_dict = {"bag_name": input_bag_name,
                            "timestamp": time_of_message,
                            "count": msg_count,
                            "x_global": light.pose.pose.position.x,
                            "y_global": light.pose.pose.position.y,
                            "z_global": light.pose.pose.position.z,
                            "state": light.state}
                tl_data_list.append(pd.DataFrame(data=[msg_dict]))
        msg_count+=1

    msg_count = 0
    for topic, msg, time_of_message in input_bag:
        if topic=="/image_color":
            cv_image = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            image_name = 'tl_data/'+input_bag_name+'_img_'+str(time_of_message)+'_count_'+str(msg_count)+'.jpg'
            cv2.imwrite(image_name, cv_image)
        msg_count+=1

    print ('Bag successfully analyzed!')

if __name__ == '__main__':
    cmd("mybag.bag")
    save("mybag.bag")

