#!/usr/bin/env python

from rospy_message_converter import json_message_converter
from std_msgs.msg import String
import rosbag

from datetime import datetime
import math

lwheel = '/makeblock/lwheel'
rwheel = '/makeblock/rwheel'
laser = '/scan'

'''
The raw log files have this format:

    {  "timestamp": [949, 943101], 
       "readings": [ ...],  
       "theta": [ ... ], 
       "right": 0, 
       "left": 0, 
       "leftTimestamp": [ 949, 940807], 
       "rightTimestamp": [ 949, 942685] }

where:

* ``readings`` is the array of range-finder readings.
* ``theta`` is the direction of each reading.
* ``left`` and ``right`` are the encoder readings, in ticks.
* ``timestamp`` is a two-shorts UNIX timestamp of when the laser data was taken; 
*   Time is supposed to be represented by a two values array, an int for seconds and another for microseconds
* ``leftTimestamp``, ``rightTimestamp`` are the timestamps for when the 
  the encoder data was taken.
'''

'''gets a rospy timestamp, return a two-short time tuple'''


def getTimeTuple(stamp):
    dt_object = datetime.fromtimestamp(stamp.to_time())
    # print("type(dt_object) =", type(dt_object))
    tuple1 = dt_object.hour * 3600 + dt_object.minute * 60 + dt_object.second
    tuple2 = dt_object.microsecond
    # print(tuple1, tuple2)
    return [tuple1, tuple2]


def normalizeAngle(angle):
    newAngle = angle
    while newAngle <= 0.:
        newAngle = newAngle + (2 * math.pi)
    return newAngle


def getThetas(msg):
    theta0 = msg.angle_min
    thetaf = msg.angle_max
    totalInterval = thetaf - theta0
    incTheta = msg.angle_increment
    accInterval = 0.
    thetas = []

    theta = normalizeAngle(theta0)

    while accInterval < totalInterval:
        # print(theta)
        thetas.append(theta)
        accInterval = accInterval + incTheta
        theta = normalizeAngle(theta - incTheta)
    return thetas


def loadBag(filename, topics):
    bag = rosbag.Bag(filename)
    for topic, msg, t in bag.read_messages(topics=topics):
        #print msg
        stamp = msg.header.stamp
        time = getTimeTuple(stamp)
        print(time)

        thetas = getThetas(msg)
        json_str = json_message_converter.convert_ros_message_to_json(msg)
        # print(t, json_str)

    bag.close()


if __name__ == "__main__":
    message = String(data='Hello')
    json_str = json_message_converter.convert_ros_message_to_json(message)

    loadBag(filename='../data/rlwheels-laser.bag', topics=[laser])
    #loadBag(filename='../data/rlwheels-laser.bag', topics=[lwheel, rwheel])
