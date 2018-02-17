#!/usr/bin/env python

""" 
The following code is a simple version of mavCommander, allowing to manually 
set the setpoint coordinates for the vehicle to fly to.
"""

import rospy
import sys
import time
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from mavState import MavState
from tf.transformations import quaternion_from_euler

# Set up ROS:
rospy.init_node('commander_node')
lpsp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=2) #for px4
lpsp_msg = PoseStamped()
lpvel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) #for px4
lpvel_msg = TwistStamped()

debug = True
rate = rospy.Rate(20)
mavState = MavState(rate,debug)
print "ROS-node is set up"

#setpoint array in ENU
'''
spArray =[[0,0,1.5],
          [0,-2,1.5],[0,-2,0.8],[0,-2,1.5],
          [2,-2,1.5],[2,-2,0.8],[2,-2,1.5],
          [2,0,1.5], [2,0,0.8],[2,0,1.5],
          [0,0,1.5], [0,0,0.8] ]
'''
spArray =[[0,0,0.8], [2,0,0.8] ]

spArrayLen = len(spArray)
spI = 0


def receive_sp():
    pass


def sp_update(data):
    #subscribe to topic, read and send data
    global spI,spArray, curVel
    #ch6swVal = int(rcindata.channels[5])
    #check distance to current setpoint
    lpx = data.pose.position.x #East 
    lpy = data.pose.position.y #North 
    lpz = data.pose.position.z #Up
    if spI==spArrayLen:
        spI=0
    spx = spArray[spI][1] #changed
    spy = spArray[spI][0] #changed
    spz = spArray[spI][2]
    #print 'lpx = ', lpx, 'spx = ',spx
    #print 'lpy = ', lpy, 'spy = ',spy
    dist2sp = math.sqrt( ((lpy-spy)**2) + ((lpx-spx)**2) + (((lpz*-1)-spz)**2) ) 
    
    print 'current setpoint index = ', spI 
    print 'current distance to setpoint = ', dist2sp
    
    if dist2sp<0.2:
        #vel_publish(0,0,0)
        if curVel<0.05:
            spI=spI+1
            print 'SWITCHED SETPOINT!!!'
            return
    else:
        sp_publish(spI)
        
       
    

def vel_update(data):
    global curVel
    velx = data.twist.linear.x
    vely = data.twist.linear.y
    velz = data.twist.linear.z
    curVel = math.sqrt(velx**2+vely**2+velz**2)
    #print 'Current velocity: ', curVel

def vel_publish(x,y,z):
    lpvel_msg.header.stamp = rospy.Time.now()
    lpvel_msg.header.frame_id = 'fcu'
    lpvel_msg.twist.linear.x = x
    lpvel_msg.twist.linear.y = y
    lpvel_msg.twist.linear.z = z
    lpvel_pub.publish(lpvel_msg)
    
def TopicListener():
    #rospy.Subscriber("mavros_msgs/RCIn", PoseStamped, sp.update)
    rospy.Subscriber("mavros/local_position/velocity", TwistStamped, vel_update)
    rospy.Subscriber("mavros/vision_pose/pose", PoseStamped, sp_update)
    rospy.spin()

def sp_publish(spI):
    global spArray
    lpsp_msg.header.stamp = rospy.Time.now()
    lpsp_msg.header.frame_id = 'fcu'
    lpsp_msg.pose.position.x    = spArray[spI][0] #East
    lpsp_msg.pose.position.y    = spArray[spI][1]  #North
    lpsp_msg.pose.position.z    = spArray[spI][2]  #Up
    lpsp_msg.pose.orientation.x = 0
    lpsp_msg.pose.orientation.y = 0
    lpsp_msg.pose.orientation.z = 0
    lpsp_msg.pose.orientation.w = 1
    lpsp_pub.publish(lpsp_msg)
    
    print spArray[spI][0],spArray[spI][1],spArray[spI][2]


if mavState.waitForFCUConnection():
    while not rospy.is_shutdown():
        TopicListener()
        sp_publish(spI)
        rate.sleep()
        
       
        
    
    