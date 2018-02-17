#!/usr/bin/env python

import rospy
import sys
import time
import math
from optparse import OptionParser
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped

from tf.transformations import quaternion_from_euler
from mavState import MavState
from mavHandler import MavHandler
from mavProgram import MavProgram


# Handling options
usage = "usage: rosrun sp_ctrl %prog [options]"
parser = OptionParser(usage=usage)
parser.set_defaults(port="/dev/ttyUSB0", debug=False)
parser.add_option("-p", "--port", action="store", type="string", dest="port", help="specify used port [default: %default]")
parser.add_option("-d", "--debug", action="store_true", dest="debug", help="print debug information")
(options, args) = parser.parse_args()

rospy.init_node('commander_node')
rate = rospy.Rate(20)

mavState = MavState(options.debug,rate)
mavProgram = MavProgram(options.debug,rate,mavState)
mavHandler = MavHandler(options.debug,rate,mavState,mavProgram)


if mavState.waitForFCUConnection():

    mavHandler.init_offset()
    
    if mavProgram.wait_for_program():
        time.sleep(0.5)
        mavHandler.setup_timers()

        while not rospy.is_shutdown():
            
            mavHandler.run_program()
            rate.sleep()



