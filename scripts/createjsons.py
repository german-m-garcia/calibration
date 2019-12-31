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

def getRangesValid(msg):
    ranges = []
    valid = []
    for range in msg.ranges:
        if ( range == float("inf")):
            ranges.append( 0.00001 )
            valid.append( 0 )
        else:
            ranges.append( range )
            valid.append( 1 )
    return ranges, valid

def getThetas(msg):
    theta0 = msg.angle_min
    thetaf = msg.angle_max
    totalInterval = thetaf - theta0
    incTheta = msg.angle_increment
    accInterval = 0.
    thetas = []

    theta = normalizeAngle(theta0)

    while accInterval < totalInterval:
        # ^print(theta)
        thetas.append(theta)
        accInterval = accInterval + incTheta
        theta = normalizeAngle(theta - incTheta)

    theta = normalizeAngle(thetaf)
    thetas.append(theta)
    return thetas

def getSortedLaserData(thetas, ranges, valid):
    
    rethetas = [None] * len( thetas )
    reranges = [None] * len( ranges )
    revalid = [None] * len( valid )
    n = len( thetas )
    indx = 0
    for i in range(181, -1, -1):        
        rethetas[indx] = thetas[i]
        reranges[indx] = ranges[i]
        revalid[indx] = valid[i]
        indx = indx + 1
    
    for i in range(359, 181, -1):
        
        rethetas[indx] = thetas[i]
        reranges[indx] = ranges[i]
        revalid[indx] = valid[i]
        indx = indx + 1
   
    return rethetas, reranges, revalid

def getStringFromList(arr):
    stringified = ""
    for i in range(0, len(arr) -2 ):
        x = arr[i]
        stringified = stringified + "{:10.7f}".format(x) + ','
    
    x = arr[ len(arr) -1 ]
    stringified = stringified + "{:10.7f}".format(x)
    
    
    return stringified
    

def getJsonFromLaserMsg( msg ):
    stamp = msg.header.stamp
    time = getTimeTuple(stamp)
    
    thetas = getThetas(msg)
    ranges, valid = getRangesValid(msg)

    # rearrange the arrays so that they go increasing from 0 to 359
    thetas, ranges, valid = getSortedLaserData(thetas, ranges, valid)
    getStringFromList(thetas)


    minTheta = thetas[0]
    maxTheta = thetas[len(thetas) -1]
    x = '"timestamp": ' + str(time)
    x = x + ', "nrays": ' + str(len(ranges))
    x = x + ', "min_theta":' + str(minTheta) + ', "max_theta":' + str(maxTheta)
    x = x + ', "theta": ' + str(thetas)
    x = x + ', "valid": ' + str(valid)
    x = x + ', "readings": ' + str(ranges)

    return x   

def getJsonFromWheelMsg( msg, left ):
    stamp = msg.header.stamp
    time = getTimeTuple(stamp)
    print time
    print msg.ticks

    if left:
        x = '"left": '+ str(msg.ticks) + ', "leftTimestamp": '+ str(time) + ''
    else:
        x = '"right": '+ str(msg.ticks) + ', "rightTimestamp": '+ str(time) + ''

    return x

def multiplexMsgs( filename, topics ):
    bag = rosbag.Bag(filename)
    lasers = []
    rwheels = []
    lwheels = []
    for topic, msg, t in bag.read_messages(topics=topics):
        
        if topic == laser:
            lasers.append( msg )

        if topic == lwheel:
            lwheels.append( msg )

        if topic == rwheel:
            rwheels.append( msg )

    bag.close()
    x = {}
    x['lasers'] = lasers
    x['lwheels'] = lwheels
    x['rwheels'] = rwheels
    return x


def getClosestWheelMsg( lwheels, laserTime):
    
    minIndex = 0
    minValue = 99999.
    i = 0
    for msg in lwheels:
        d = abs(msg.header.stamp.to_sec() - laserTime.to_sec())
        if d < minValue:
            minIndex = i
            minValue = d
        i = i + 1
    
    return lwheels[minIndex]


def filterWheelMsgs(muxMsgs):
    lasers = muxMsgs['lasers']
    lwheels = muxMsgs['lwheels']
    rwheels = muxMsgs['rwheels']
    filteredRWheels = []
    filteredLWheels = []
    for laserMsg in lasers:
        laserTime = laserMsg.header.stamp
        lwheelMsg = getClosestWheelMsg( lwheels, laserTime)
        rwheelMsg = getClosestWheelMsg( rwheels, laserTime)
        filteredLWheels.append(lwheelMsg)
        filteredRWheels.append(rwheelMsg)
    
    x = {}
    x['lasers'] = lasers
    x['lwheels'] = filteredLWheels
    x['rwheels'] = filteredRWheels
    return x

def getJsonFromMsgs(laserMsg, rwheelMsg, lwheelMsg):
    
    leftStamp = lwheelMsg.header.stamp
    leftTime = getTimeTuple(leftStamp)
    
    rightStamp = rwheelMsg.header.stamp
    rightTime = getTimeTuple(rightStamp)

    laser = '{' + getJsonFromLaserMsg(laserMsg)
    
    x = ', "right": ' + str(rwheelMsg.ticks) + ', "left": '+ str(lwheelMsg.ticks)
    x = x + ', "leftTimestamp": '+ str(leftTime) + ', "rightTimestamp": '+ str(rightTime)
    x = x + ', "odometry": [0.,0.,0.]}'
    x = laser + x
    return x

def loadBag(filename, topics):
    #bag = rosbag.Bag(filename)
    lasers = []
    rwheels = []
    lwheels = []

    muxMsgs = multiplexMsgs(filename, topics)
    # 3 parallel arrays with laser, rwheel, lwheel topics
    muxMsgs = filterWheelMsgs( muxMsgs )

    len1 =  len( muxMsgs['lasers'] )
    len2 =  len(muxMsgs['lwheels'])
    len3 =  len(muxMsgs['rwheels'])

    if(not len1 == len2 == len3):
        print 'topics lengths do not match', len1, len2, len3

    print 'writing ', len1, ' topics to json file'

    json_str = ''
    for (laserMsg, lWheelMsg, rWheelMsg) in zip(muxMsgs['lasers'],muxMsgs['lwheels'], muxMsgs['rwheels'] ):
        jsonEntry = getJsonFromMsgs(laserMsg, lWheelMsg, rWheelMsg)
        json_str = json_str + jsonEntry
    

    with open("tmp.json", "w") as text_file:
        text_file.write(json_str)

    print 'done'
    """for topic, msg, t in bag.read_messages(topics=topics):
        #print topic
        #print msg
        
        if topic == laser:            
            laserJson = getJsonFromLaserMsg( msg )
            lasers.append( laserJson )
        
        if topic == lwheel:            
            lWheelJson = getJsonFromWheelMsg( msg, True )
            print lWheelJson
            lwheels.append( lWheelJson )

        if topic == rwheel:
            rWheelJson = getJsonFromWheelMsg( msg, False )
            print rWheelJson
            rwheels.append( rWheelJson )

        
        #json_str = json_message_converter.convert_ros_message_to_json(msg)"""

    #bag.close()


if __name__ == "__main__":
    message = String(data='Hello')
    json_str = json_message_converter.convert_ros_message_to_json(message)

    loadBag(filename='../data/rlwheels-laser-2.bag', topics=[laser, lwheel, rwheel])
    #loadBag(filename='../data/rlwheels-laser.bag', topics=[laser,lwheel, rwheel])
