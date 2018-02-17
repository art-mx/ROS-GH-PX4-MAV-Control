#!/usr/bin/env python

import rospy
import sys
import time
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Pose
from mavros_msgs.msg import State

def state_update(data):
    
    global isConnected
    isConnected = data.connected
    isArmed = data.armed
    mode = data.mode
    #rospy.loginfo( 'Updated state: '+'MODE: '+str(mode) )
#    

def gh_setpoint_update(data):
    print 'gh_setpoint_update'
    global targetX,targetY,targetZ

    targetX = float(data.position.x)
    targetY = float(data.position.y)
    targetZ = float(data.position.z)
    rospy.logwarn( 'INFO: '+'Received new destination!'+'\n'+str(targetX)+', '+str(targetY)+', '+str(targetZ) )
#
def publish_target_pose():
    
    global targetX,targetY,targetZ,loop_count
    
    setpoint_msg.header.stamp = rospy.Time.now()
    setpoint_msg.header.seq = loop_count
    setpoint_msg.header.frame_id = 'fcu'
    
    setpoint_msg.pose.position.x    = targetX
    setpoint_msg.pose.position.y    = targetY
    setpoint_msg.pose.position.z    = targetZ
    
    setpoint_msg.pose.orientation.x = 0
    setpoint_msg.pose.orientation.y = 0
    setpoint_msg.pose.orientation.z = 0
    setpoint_msg.pose.orientation.w = 1
    
    setpoint_pub.publish(setpoint_msg)
#
def waitForFCUConnection():

    rospy.logwarn("Waiting for FCU connection...")
    while not rospy.is_shutdown():
        if isConnected:
            rospy.logwarn("FCU is connected...")
            return True
        rate.sleep()
    rospy.logwarn("ROS is shutdown")
    return False
#

rospy.init_node('commander_node')

rospy.Subscriber('/mavros/state', State, state_update)
rospy.Subscriber('/gh/targetpose', Pose, gh_setpoint_update)

setpoint_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=2)
setpoint_msg = PoseStamped()

rate = rospy.Rate(20)
loop_count = 0
targetX,targetY,targetZ = 0,0,0
isConnected = 0

waitForFCUConnection()

while not rospy.is_shutdown():

    publish_target_pose()
          
    loop_count+=1  
    rate.sleep()
        
        
        
        
        