import rospy
from mavros_msgs.msg import State
from mavros_msgs.msg import ExtendedState
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from mavros_msgs.msg import RCIn



class MavState:
	
	def __init__(self,debug,rate):
		rospy.Subscriber('/mavros/state', State, self.stateCb)
		rospy.Subscriber('/mavros/extended_state', ExtendedState, self.extstateCb)
		rospy.Subscriber('/mavros/rc/in', RCIn, self.rcin_update)
		
		self.modeHandle = rospy.ServiceProxy("/mavros/set_mode", SetMode)
		self.armHandle= rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		self.landHandle= rospy.ServiceProxy('/mavros/cmd/land', CommandTOL)

		self.currentState = State()
		self.extState = ExtendedState()
		self.rate = rate
		self.debug = debug
		self.rcResumeSw = False		
		
		#self.offbTimerThr = rospy.Duration(2.8) # 2 second window for setting offboard, otherwise disarm
	#eof

	def stateCb(self, data):
		#if self.debug: print 'state callback: ', data
		self.currentState = data
	#eof
	
	def extstateCb(self, data):
		self.extState = data
	#eof
	
	def rcin_update(self,data):
		rc = data.channels
		if rc != None :
			if rc[6] > 1500:
				if self.debug: rospy.logwarn("Resume switch ENGAGED...")
				self.rcResumeSw = True
		else: 
			rospy.logwarn('No RC data available! Check remote!')
			self.rcResumeSw = False
	#callback 	
	
	def isLanded(self):
		#STATE_UNDEFINED = 0
		#STATE_ON_GROUND = 1
		#STATE_IN_AIR = 2
		if self.extState.landed_state == 1:
			return True
		else: return False

	def wait_for_landed(self):
		rospy.loginfo("Waiting for Landing... ")
		while not rospy.is_shutdown():
			if self.isLanded():
				rospy.logwarn("Vehicle is LANDED!!!")
				return True
			self.rate.sleep()
	#	
	
	def wait_for_armed(self):
		rospy.loginfo("Waiting for Arming... Please ARM the vehicle")
		while not rospy.is_shutdown():
			if self.isArmed():
				rospy.logwarn("Vehicle is ARMED!!!")
				return True
			self.rate.sleep()
	#	
	
	def wait_for_disarmed(self):
		rospy.loginfo("Waiting for disarming... Please DISARM the vehicle")
		while not rospy.is_shutdown():
			if not self.isArmed():
				rospy.logwarn("Vehicle is DISARMED!!!")
				return True
			self.rate.sleep()
	#	
	
	
	def wait_for_mode(self,mode,timeout=60): # max time to wait for state is 1 minute
		rospy.loginfo('Waiting for '+str(mode)+'...')
		timerStart = rospy.Time.now()
		
		while not rospy.is_shutdown():
			if rospy.Time.now() - timerStart > rospy.Duration(timeout):
				rospy.logwarn(str(mode)+' activation TIMEOUT!!!')
				return False
			
			if self.doSetMode(mode):
				return True
			self.rate.sleep()
	#		
	
	def wait_for_offboard(self,timeout):
		rospy.loginfo("Waiting for OFFBOARD...")
		timerStart = rospy.Time.now()
		
		while not rospy.is_shutdown():
			if rospy.Time.now() - timerStart > rospy.Duration(timeout):
				rospy.logwarn("OFFBOARD activation TIMEOUT!!!")
				#self.doDisarm()
				return False
			if self.isOffboard():
				rospy.logwarn("OFFBOARD Activated!!!")
				return True
			self.rate.sleep()
	#	
			
	def waitForFCUConnection(self):
	
		if not self.isConnected(): 
			rospy.loginfo("Waiting for FCU connection...")
			while not rospy.is_shutdown():
				if self.isConnected():
					rospy.logwarn("FCU is connected...")
					return True
		else: 
			rospy.logwarn("FCU is connected...")
			return True
	

			
			self.rate.sleep()
		
		rospy.logwarn("ROS is shutdown")
		return False
	#eof

	def isConnected(self):
		return self.currentState.connected
	#eof

	def isArmed(self):
		#if self.debug: print 'armed state:', self.currentState.armed
		return self.currentState.armed
	#eof

	def isOffboard(self):
		return self.isConnected() and self.currentState.mode == "OFFBOARD"
	#eof
	
	def doLand(self):
		if self.currentState.armed: #
			rospy.logwarn('calling LAND SERVICE...')
			rospy.wait_for_service("/mavros/cmd/land")
			
			try:
			    response = self.landHandle()
			    print 'response: ',response
			    if response.success: 
			    	rospy.logwarn('LANDING!!!')
			    	return True
			 	if not response.success: 
			 		rospy.logwarn('Land service FAILED!!!')
			 		return False
			except rospy.ServiceException, e:
			    print "Land Service call failed: %s"%e
	#		
		
	
	def doArm(self):
		if not self.currentState.armed: #
			rospy.logwarn('calling ARM service...')
			rospy.wait_for_service("/mavros/cmd/arming")
			
			try:
			    response = self.armHandle(True)
			    print 'response: ',response
			    if response.success: 
			    	rospy.logwarn('ARMED!!!')
			    	return True
			 	if not response.success: 
			 		rospy.logwarn('ARMING FAILED!!!')
			 		return False
			except rospy.ServiceException, e:
			    print "Arm Service call failed: %s"%e
	#
	
	def doDisarm(self):
		if 	self.currentState.armed:
			rospy.logwarn('calling DISARM service...')
			rospy.wait_for_service("/mavros/cmd/arming")
			try:
			    response = self.armHandle(False)
			    if response.success: 
			    	rospy.logwarn('DISARMED!!!')
			    	return True
			 	if not response.success: 
			 		rospy.logwarn('DISARMING FAILED!!! SHUTTING DOWN!!!')
			 		rospy.signal_shutdown()
			 		sys.exit()
			 		return False			    
			except rospy.ServiceException, e:
			    print "Disarm Service call failed: %s"%e
	#
			
	
	def doSetMode(self,mode):
 
		rospy.loginfo('Setting '+str(mode)+' mode...')
		rospy.wait_for_service("/mavros/set_mode")
		try:
			response = self.modeHandle(0,mode)
			if response.success:
				rospy.logwarn(str(mode)+' mode SET SUCCESSFULLY!!!')
				return True
			else:
				rospy.logwarn(str(mode)+' mode SET FAILED!!!')
				return False
		except rospy.ServiceException, e:
			print "Disarm Service call failed: %s"%e

	#

	
	
