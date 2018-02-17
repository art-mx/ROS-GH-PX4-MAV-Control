#!/usr/bin/env python

import rospy
import sys
import time
import math
from geometry_msgs.msg import PoseStamped
#from geometry_msgs.msg import TwistStamped #add to package.xml
from mavState import MavState
from tf.transformations import quaternion_from_euler

# Set up ROS:
rospy.init_node('commander_node')
lpsp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=2) #for px4
lpsp_msg = PoseStamped()
#lpvel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) #for px4
#lpvel_msg = PoseStamped()
print "ROS-nodes is set up"

#setpoint array in ENU
'''
spArray = [ [0,0,-0.2],[0,0,1.5],
        [0,-2,1.5],[0,-2,-0.5],[0,-2,1.5],
        [2,-2,1.5],[2,-2,-0.5],[2,-2,1.5],
        [2,0,1.5],[2,0,-0.5],[2,0,1.5] ]
'''
#generate setpoint array
spArray=[]
cirR=1
for a in range(0,360):
    print a
    spArray.append( [cirR*math.cos(a*0.0175),cirR*math.sin(a*0.0175),1.5] )   
    print spArray[-1][0] 
#spArrayLen = len(spArray)
spArrayLen = 360
spI = 0
spRes = 10 #use every nth setpoint


#subscribe to topic, read and send data
def sp_update(data):

    global spI, spArray
    #ch6swVal = int(rcindata.channels[5])
    #check check distance to current setpoint
    lpx = data.pose.position.x #East
    lpy = data.pose.position.y #North
    lpz = data.pose.position.z #Up
    if spI==spArrayLen:
        spI=0
    spx = spArray[spI][0]
    spy = spArray[spI][1]
    spz = spArray[spI][2]
    print 'lpx = ', lpx, 'spx = ',spx
    print 'lpy = ', lpy, 'spy = ',spy
    dist2sp = math.sqrt( ((lpy-spx)**2) + ((lpx-spy)**2) + (((lpz*-1)-spz)**2) )
    #if close, increment setpoint index
    if dist2sp<0.1:
        spI=spI+spRes
        print 'SWITCHED SETPOINT!!!'
        print 'current setpoint index = ', spI 
        print 'curent distance to setpoint = ', dist2sp

    sp_publish(spI)   
    
    
def TopicListener():
    
    #rospy.Subscriber("mavros_msgs/RCIn", PoseStamped, sp.update)
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

rate = rospy.Rate(20)

mavState = MavState(rate)


if mavState.waitForFCUConnection():
    while not rospy.is_shutdown():
        TopicListener()
        
        rate.sleep()
        
       
        
    
    