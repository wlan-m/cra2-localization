#!/usr/bin/env python2.7

import os
import rospy
from duckietown import DTROS
from std_msgs.msg import String
import numpy as np
import rosbag
import os
from os import listdir
from os.path import isfile, join


def analyzer():
    print(os.getcwd())
    mypath = os.getcwd()
    onlyfiles = [f for f in listdir(mypath) if isfile(join(mypath, f))]
    print(onlyfiles)
    bag1 = rosbag.Bag('example_rosbag_H3.bag')
    bag2 = rosbag.Bag('3_param10.bag')
    print(bag1)
    print(bag2)
    #for topic, msg, in bag.read_messages():
        #print ('WTF')
        #print(msg)


'''
# choose which recoding you want to analyse:
choice = 0      # choice = 0 --> example_rosbag_H3; choice = 1 --> my own recording

if choice == 0:
    bag = rosbag.Bag('/code/catkin_ws/src/my-ex19-program/packages/my_package/rosbags/example_rosbag_H3.bag')

    print "choice: analyse example_rosbag_H3.bag"

    # topic: camera_info
    cam_info = []

    for topic, msg, t in bag.read_messages(topics=['/tesla/camera_node/camera_info']):
        cam_info.append(t)

    cam_info_dif = np.diff(cam_info)
    num_messages = len(cam_info)
    num_messages_dif = len(cam_info_dif)
    min_period = min(cam_info_dif)
    max_period = max(cam_info_dif)
    avg_period = np.sum(cam_info_dif).to_sec()/float(num_messages_dif)
    med_period = np.median(cam_info_dif)

    print "/tesla/camera_node/camera_info: " + "num_messages: " + str(num_messages) + " period: min: " + str(round(min_period.to_sec(),2)) + " max: " +  str(round(max_period.to_sec(),2)) + " average: " + str(round(avg_period,2)) + " median: " + str(round(med_period.to_sec(),2))

    # topic: segment_list
    cam_info = []

    for topic, msg, t in bag.read_messages(topics=['/tesla/line_detector_node/segment_list']):
        cam_info.append(t)

    cam_info_dif = np.diff(cam_info)
    num_messages = len(cam_info)
    num_messages_dif = len(cam_info_dif)
    min_period = min(cam_info_dif)
    max_period = max(cam_info_dif)
    avg_period = np.sum(cam_info_dif).to_sec()/float(num_messages_dif)
    med_period = np.median(cam_info_dif)

    print "/tesla/line_detector_node/segment_list: " + "num_messages: " + str(num_messages) + " period: min: " + str(round(min_period.to_sec(),2)) + " max: " +  str(round(max_period.to_sec(),2)) + " average: " + str(round(avg_period,2)) + " median: " + str(round(med_period.to_sec(),2))

    # topic: wheels_cmd
    cam_info = []

    for topic, msg, t in bag.read_messages(topics=['/tesla/wheels_driver_node/wheels_cmd']):
        cam_info.append(t)

    cam_info_dif = np.diff(cam_info)
    num_messages = len(cam_info)
    num_messages_dif = len(cam_info_dif)
    min_period = min(cam_info_dif)
    max_period = max(cam_info_dif)
    avg_period = np.sum(cam_info_dif).to_sec()/float(num_messages_dif)
    med_period = np.median(cam_info_dif)

    print "/tesla/wheels_driver_node/wheels_cmd: " + "num_messages: " + str(num_messages) + " period: min: " + str(round(min_period.to_sec(),2)) + " max: " +  str(round(max_period.to_sec(),2)) + " average: " + str(round(avg_period,2)) + " median: " + str(round(med_period.to_sec(),2))

    bag.close()


elif choice == 1:
     bag = rosbag.Bag('/code/catkin_ws/src/my-ex19-program/packages/my_package/rosbags/amod19-rh3-ex-record-matthiaswieland.bag')

     print "choice: analyse amod19-rh3-ex-record-matthiaswieland.bag"

     #print (bag)
     #for topic, msg, t in bag.read_messages(topics=['/tesla/camera_node/camera_info', '/tesla/line_detector_node/segment_list', '/tesla/wheels_driver_node/wheels_cmd']):
     #     print topic
     #    #print("wtf is going on")
     #

     # topic: camera_info
     cam_info = []

     for topic, msg, t in bag.read_messages(topics=['/wlan/camera_node/image/compressed']):
         cam_info.append(t)

     cam_info_dif = np.diff(cam_info)
     num_messages = len(cam_info)
     num_messages_dif = len(cam_info_dif)
     min_period = min(cam_info_dif)
     max_period = max(cam_info_dif)
     avg_period = np.sum(cam_info_dif).to_sec() / float(num_messages_dif)
     med_period = np.median(cam_info_dif)

     print "/wlan/camera_node/image/compressed: " + "num_messages: " + str(num_messages) + " period: min: " + str(
         round(min_period.to_sec(), 2)) + " max: " + str(round(max_period.to_sec(), 2)) + " average: " + str(
         round(avg_period, 2)) + " median: " + str(round(med_period.to_sec(), 2))

     # topic: segment_list
     cam_info = []

     for topic, msg, t in bag.read_messages(topics=['/wlan/wheels_driver_node/wheels_cmd']):
         cam_info.append(t)

     cam_info_dif = np.diff(cam_info)
     num_messages = len(cam_info)
     num_messages_dif = len(cam_info_dif)
     min_period = min(cam_info_dif)
     max_period = max(cam_info_dif)
     avg_period = np.sum(cam_info_dif).to_sec() / float(num_messages_dif)
     med_period = np.median(cam_info_dif)

     print "/wlan/wheels_driver_node/wheels_cmd: " + "num_messages: " + str(num_messages) + " period: min: " + str(
         round(min_period.to_sec(), 2)) + " max: " + str(round(max_period.to_sec(), 2)) + " average: " + str(
         round(avg_period, 2)) + " median: " + str(round(med_period.to_sec(), 2))


'''


if __name__ == '__main__':
    # create the node
    try:
        analyzer()
    except rospy.ROSInterruptException:
        pass