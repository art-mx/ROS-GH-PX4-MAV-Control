import rospy
import numpy
import sys
import time
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import String
from std_msgs.msg import ColorRGBA

class MavHandler:
    
    def __init__(self,debug,rate,mavState,mavProgram):
        self.lpsp_pub = rospy.Publisher("/mavros/setpoint_position/local", PoseStamped, queue_size=2) 
        self.lpvel_pub = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel", TwistStamped, queue_size=2) 
        self.vsn_pub = rospy.Publisher("/mavros/vision_pose/pose", PoseStamped, queue_size=2)
        self.led_pub = rospy.Publisher("/arduino/led", ColorRGBA, queue_size=1)
        self.debug_pub = rospy.Publisher("/debug", String, queue_size=1)
        
        rospy.Subscriber("/mavros/local_position/velocity", TwistStamped, self.lpvel_update)
        rospy.Subscriber("/mavros/local_position/pose", PoseStamped, self.lpos_update)
        rospy.Subscriber("/leica/radio/worldposition", PointStamped, self.vsn_update)
        
        self.lpsp_msg = PoseStamped()
        self.lpvel_msg = TwistStamped()
        self.vsn_msg = PoseStamped()
        
        self.lposx = 0.0
        self.lposy = 0.0
        self.lposz = 0.0
        
        #self.offx = 0.0
        #self.offy = 0.0
        #self.offz = 0.0
        
        self.initPos = [0.0,0.0,0.0]
         # start callback functions after the program is loaded
        self.pub_rate = rospy.Duration(0.05) #20hz 
        self.info_rate = rospy.Duration(1) #3hz
        self.high_update_rate = rospy.Duration(0.02) #50hz

        self.debug = debug
        self.rate = rate
        self.waitrate = rospy.Rate(0.3)
        
        self.mavState = mavState
        self.mavProgram = mavProgram

        #self.curVel = 0.0
        self.takeoffVel = [0.0, 0.0, 5.0] 
        self.desiredVel = [0.0, 0.0, 0.0]
        self.landVel = [0.0, 0.0, -1.0]
        self.zeroVel = [0.0 ,0.0 ,0.0 ]
        self.velStabThr = 0.3 # considered a stable velocity condition
        
        self.takeoffTarget = [0.0, 0.0, 0.0]
        self.takeoffLoc = [0.0, 0.0, 0.0]
        
        self.vsnxCor = 0.0
        self.vsnyCor = 0.0
        self.vsnzCor = 0.0
        
        self.spxCor = 0.0
        self.spyCor = 0.0
        self.spzCor = 0.0
        
        self.trajTrgtAvgN = 10
        self.trajThreshDist = 0.6
        self.trajTrgtAvgX = 0.0
        self.trajTrgtAvgY = 0.0
        self.trajTrgtAvgZ = 0.0
        
        self.vsnx = 0.0
        self.vsny = 0.0
        self.vsnz = 0.0

        self.vsnxPrev = 0.0
        self.vsnyPrev = 0.0
        self.vsnzPrev = 0.0

        self.velx = 0.0
        self.vely = 0.0
        self.velz = 0.0

        self.spDistZ = 0.0
        self.spDistXY = 0.0
        
        self.spDistZthr = 0.1 # meters
        self.spDistXYthr = 0.3 # meters
        self.spDistThr = 0.3
        
        self.takeoffTimerThr = rospy.Duration(5.0) # seconds
        self.visionTimeotThr = 2.0
        self.visionValid = True
        #self.takeoffTrigger = False
        self.landTrigger = True
        #self.resumeTrigger = False  
        rospy.Timer(self.pub_rate, self.vsn_publish)      
        self.vsnTrigger = False # start with false
        self.velTrigger = False    
        
        
        self.overrideLed = [None,None,None]
        
    #

    def setup_timers(self):
        rospy.Timer(self.pub_rate, self.vel_publish)
        rospy.Timer(self.pub_rate, self.sp_publish)
        #rospy.Timer(self.info_rate, self.print_info)
        rospy.Timer(rospy.Duration(0.33), self.check_vision_stream)
        rospy.Timer(rospy.Duration(0.25), self.led_publish)   
        rospy.Timer(rospy.Duration(0.5), self.debug_publish)
        
    #
    
    def init_offset(self):
        time.sleep(0.5)
        self.check_vision_stream()
        rospy.loginfo('INITIALIZING OFFSETS! ')
        self.vsnTrigger = False
        self.initPos = [self.vsnx, self.vsny, self.vsnz]
        rospy.loginfo('Start position ENU:'+' '+str(self.vsnx)+' '+str(self.vsny)+' '+str(self.vsnz) ) 
        rospy.loginfo('Will be offsetting setpoints and vision to that position')
        self.vsnTrigger = True
        # TODO wait for local pose to reset to 0
        rospy.logwarn('Waiting for estimator reset...')
        
        while not rospy.is_shutdown():
            if self.lposx < 0.1 and self.lposy < 0.1 and self.lposz < 0.1:
                rospy.logwarn('Estimator reset to zero! Proceeding...')
                
                return True
            self.rate.sleep()
    #  
      
    def run_program(self):

        if self.mavProgram.seqType[self.mavProgram.seqIdCur] == 'T':
            self.sp_takeoff()
            
        elif self.mavProgram.seqType[self.mavProgram.seqIdCur] == 'Mt':
            self.traj_follow()
            
        elif self.mavProgram.seqType[self.mavProgram.seqIdCur] == 'M':
            self.sp_move()
            

        elif self.mavProgram.seqType[self.mavProgram.seqIdCur] == 'L':
            self.sp_land()
    #
    

    
    def sp_takeoff(self):
        
        if not self.prepare_for_takeoff(): return
        #self.start_accelerated_takeoff()
        if not self.takeoff_at_cur_pos(): return
        self.bring_velocity_to_zero()

        self.mavProgram.next_sequence()
        return
    #
    
    def prepare_for_takeoff(self):
        # safety check
        if self.mavState.isArmed(): rospy.logfatal('Vehicle ARMED!!! Takeoff command REJECTED! Please DISARM!')
        self.mavState.wait_for_disarmed()
        
        self.init_offset() # reset estimator to zero before every takeoff
        
        i = self.mavProgram.seqIdCur
        
        self.takeoffLoc[0] = 0.0
        self.takeoffLoc[1] = 0.0
        self.takeoffLoc[2] = 0.0
        
        self.takeoffTarget[0] = 0 # init position x
        self.takeoffTarget[1] = 0 # init position y
        self.takeoffTarget[2] = self.spzCor # z setpoint target above the setpoint
        
        #self.takeoffTrigger = True # trigger xy setpoint coords to be the xy takeoff coords

        self.mavState.wait_for_armed()
        if self.mavState.wait_for_offboard(3): # IF offboard not activated in 3s, start over
            rospy.logwarn('Proceeding with takeoff to '+str(round(self.takeoffTarget[2],2))+' meters!')
            self.print_info()
            return True
        else: return False

    #


    def takeoff_at_cur_pos(self):
        
        self.velTrigger = True
        self.desiredVel = self.takeoffVel
        #timerStart = rospy.Time.now()
        while not rospy.is_shutdown():
             
            if self.vsnzCor > self.takeoffTarget[2]-self.spDistZthr:
                rospy.logwarn('Reached target altitude')
                self.print_info()
                self.velTrigger = False
                return True
            '''
            if rospy.Time.now() - timerStart > self.takeoffTimerThr:
                rospy.logfatal('Takes too long, engaging position hold and returning')
                mavState.doSetMode('POSCTL')
                return False
            '''
            self.rate.sleep()
    #
        
            

    def touchdown_setpoint(self):
        
        self.velTrigger = True
        #self.takeoffTrigger = False # unblock xy setpoint component
        
        rospy.logwarn('Landing to setpoint!'+'\n')
        self.correct_setpoint()
        self.print_info()
        
        while not rospy.is_shutdown():

            if self.sp_dist_thresh(self.spDistThr):
                self.print_info()
                rospy.logwarn('Reached target!!!' )
                self.print_info()
                return True
            self.rate.sleep()
    #   

    def bring_velocity_to_zero(self):
        rospy.loginfo('Stabilizing velocity... ' +'\n')
        self.print_info()
        self.desiredVel = self.zeroVel
        self.velTrigger = True # start sending velocity
        self.overrideLed = [0,0,255] 
        
        while not rospy.is_shutdown(): 
            if self.vel_thresh(self.velStabThr):
                rospy.logwarn('Velocity stable!'+' Requesting NEXT SEQUENCE!!!')
                self.print_info()
                self.velTrigger = False
                self.overrideLed = [None,None,None]
                return
                # TODO - request user input switch
                if self.mavState.rcResumeSw == True:
                    rospy.logwarn('Skipping VELOCITY check!!! PROCEEDING!!!')
                    self.overrideLed = [None,None,None]
                    return
            self.rate.sleep()
    #    
        
    def sp_move(self):

        #self.velTrigger = False
        #self.takeoffTrigger = False
        self.correct_setpoint()
        self.approach_setpoint()
        self.bring_velocity_to_zero()
        self.mavProgram.next_sequence()
        return
    #
    def traj_follow(self):
        
        self.correct_setpoint()
        
        while not rospy.is_shutdown():
            try:
                if self.check_traj_position(): # if reached the end of trajectory
                    self.bring_velocity_to_zero()
                    self.mavProgram.next_sequence()
                    return
                self.rate.sleep()
            except: 
                rospy.logfatal('EXCEPTION!!! LANDING!!!')
                self.mavState.doLand()
                break
        self.mavState.wait_for_disarmed()
                
            
            
    
    #
    
    
    def approach_setpoint(self):
        
        #edited#self.velTrigger = False # stop sending velocity
        #self.takeoffTrigger = False # unblock xy setpoint component
        rospy.logwarn('Moving to setpoint!'+'\n')
        #edited#self.correct_setpoint() this was already executed in sp_move
        self.print_info()
        self.overrideLed = [100,255,100]
        
        while not rospy.is_shutdown():
            if self.sp_dist_thresh(self.spDistThr):
                self.print_info()
                rospy.logwarn('Reached target!!!' )
                self.print_info()
                self.overrideLed = [None,None,None]
                return True
            self.rate.sleep()
    #
    
    def sp_land(self):
        rospy.loginfo('Proceeding with LANDING!'+'\n')
        self.overrideLed = [255,255,255]
        #self.desiredVel = self.landVel
        #self.velTrigger = True
        
        #self.touchdown_setpoint() # replace with landing handler
        self.mavState.doLand()
        #self.mavState.wait_for_landed()
        if self.vsnzCor < 0.1:
            self.mavState.doDisarm()
        self.mavState.wait_for_disarmed()
        self.overrideLed = [None,None,None]
        self.mavProgram.next_sequence()
        return
    #

    def sp_dist_thresh(self,dist):
        
        # distances in autopilot frame
        i = self.mavProgram.seqIdCur
        self.spDistX =  abs(   self.vsnxCor - self.spxCor   ) #abs(  self.vsnx -  self.mavProgram.x[i]  )
        self.spDistY =  abs(   self.vsnyCor - self.spyCor   ) #abs(  self.vsny -  self.mavProgram.y[i]  )
        self.spDistZ =  abs(   self.vsnzCor - self.spzCor   ) #abs(  self.vsnz -  self.mavProgram.z[i]  )   
             
        if self.spDistX<dist and self.spDistY<dist and self.spDistZ<dist:
            return True
        else: return False
    #
    
    def vel_thresh(self,vel):
        if abs(self.velx)<vel and abs(self.vely)<vel and abs(self.velz)<vel:
            return True
    #   
    
    def lpvel_update(self,data):
        self.velx = data.twist.linear.x
        self.vely = data.twist.linear.y
        self.velz = data.twist.linear.z
    #callback  
    
    def lpos_update(self,data):
        self.lposx = data.pose.position.x
        self.lposy = data.pose.position.y
        self.lposz = data.pose.position.z
    #callback  
    

        
    def vsn_update(self,data): 
        
        self.vsnx = data.point.x #data.point.y
        self.vsny = data.point.y #data.point.x
        self.vsnz = data.point.z #data.point.z*-1
    #callback  
    
    def led_publish(self,*args): # update LED color
        
        i = self.mavProgram.seqIdCur
        j = self.mavProgram.trajIdCur
        
        if self.overrideLed[0] != None: # check if error led value is not empty
            r,g,b = self.overrideLed
            
            self.led_pub.publish(float(r),float(g),float(b),0.0) #publish as red color
        
        else:
            if self.mavProgram.seqType[i] == 'T':
                self.led_pub.publish(0.0,0.0,255.0,0.0)        
            
            elif self.mavProgram.seqType[i] == 'M':
                r = self.mavProgram.r[i]
                g = self.mavProgram.g[i]
                b = self.mavProgram.b[i]
                self.led_pub.publish(r,g,b,0.0)        
            
            elif self.mavProgram.seqType[i] == 'Mt':
                r = self.mavProgram.r[i][j]
                g = self.mavProgram.g[i][j]
                b = self.mavProgram.b[i][j]
                self.led_pub.publish(r,g,b,0.0)
    
            elif self.mavProgram.seqType[i] == 'L':
                self.led_pub.publish(255.0,255.0,255.0,0.0)
            

        
    #
    
    def vsn_publish(self,*args):
        #print 'self.vsnTrigger: ',self.vsnTrigger
        if self.vsnTrigger:
            self.correct_vision()
            #print 'sending vision'
            self.vsn_msg.header.stamp = rospy.Time.now()
            self.vsn_msg.header.frame_id = 'fcu'
            self.vsn_msg.pose.position.x    = self.vsnyCor # corrected for autopilot buggy format
            self.vsn_msg.pose.position.y    = self.vsnxCor # corrected for autopilot buggy format
            self.vsn_msg.pose.position.z    = -self.vsnzCor # corrected for autopilot buggy format
            self.vsn_pub.publish(self.vsn_msg) 
        
    #
    def vel_publish(self,*args):
        
        if self.velTrigger:
            self.lpvel_msg.header.stamp = rospy.Time.now()
            self.lpvel_msg.header.frame_id = 'fcu'
            self.lpvel_msg.twist.linear.x = self.desiredVel[0]
            self.lpvel_msg.twist.linear.y = self.desiredVel[1]
            self.lpvel_msg.twist.linear.z = self.desiredVel[2]
            self.lpvel_pub.publish(self.lpvel_msg)
    #
    def sp_publish(self,*args):
        
        #self.correct_setpoint() # doesn't need to be here, can be executed only at new setpoint
        
        i = self.mavProgram.seqIdCur
        j = self.mavProgram.trajIdCur
        
        self.lpsp_msg.header.stamp = rospy.Time.now()
        self.lpsp_msg.header.frame_id = 'fcu'

        self.lpsp_msg.pose.position.x = self.spxCor #self.mavProgram.x[i] - self.initPos[0]
        self.lpsp_msg.pose.position.y = self.spyCor #self.mavProgram.y[i] - self.initPos[1]
        self.lpsp_msg.pose.position.z = self.spzCor #self.mavProgram.z[i] + self.initPos[2] 
        
        ########## EDIT ########## EDIT ########## EDIT 
        if self.mavProgram.seqType[i] == 'Mt':
            self.lpsp_msg.pose.orientation.x = self.mavProgram.qx[i][j]
            self.lpsp_msg.pose.orientation.y = self.mavProgram.qy[i][j]
            self.lpsp_msg.pose.orientation.z = self.mavProgram.qz[i][j]
            self.lpsp_msg.pose.orientation.w = self.mavProgram.qw[i][j]
        else:   
            self.lpsp_msg.pose.orientation.x = self.mavProgram.qx[i]
            self.lpsp_msg.pose.orientation.y = self.mavProgram.qy[i]
            self.lpsp_msg.pose.orientation.z = self.mavProgram.qz[i]
            self.lpsp_msg.pose.orientation.w = self.mavProgram.qw[i]
            
        self.lpsp_pub.publish(self.lpsp_msg)
    #
    def debug_publish(self,*args):
        i = self.mavProgram.seqIdCur
        j = self.mavProgram.trajIdCur
        
        if self.mavProgram.seqType[i] == 'Mt':
            debugString = str(self.trajTrgtAvgX)+','+str(self.trajTrgtAvgY)+','+str(self.trajTrgtAvgZ)
            self.debug_pub.publish(debugString)

        else:
            debugString = str(self.mavProgram.x[i])+','+str(self.mavProgram.y[i])+','+str(self.mavProgram.z[i])
            self.debug_pub.publish(debugString)
             
    #
    
    def correct_setpoint(self): # this should only be run once at new setpoint or traj targget
        
        i = self.mavProgram.seqIdCur
        j = self.mavProgram.trajIdCur
        # correct setpoint for initial position, ENU format
        
        if self.mavProgram.seqType[i] == 'L':
            self.spxCor = self.mavProgram.x[i] - self.initPos[0]
            self.spyCor = self.mavProgram.y[i] - self.initPos[1] 
            self.spzCor = self.mavProgram.z[i] - 0.1 - self.initPos[2] 
                    
        elif self.mavProgram.seqType[i] == 'T':
            self.spxCor = self.takeoffTarget[0]
            self.spyCor = self.takeoffTarget[1]
            self.spzCor = self.mavProgram.z[i] - self.initPos[2] 
            
        elif self.mavProgram.seqType[i] == 'M':
            self.spxCor = self.mavProgram.x[i] - self.initPos[0] 
            self.spyCor = self.mavProgram.y[i] - self.initPos[1] 
            self.spzCor = self.mavProgram.z[i] - self.initPos[2]  
            
        elif self.mavProgram.seqType[i] == 'Mt': 
            self.calc_avg_traj_target()
            self.spxCor = self.trajTrgtAvgX - self.initPos[0] 
            self.spyCor = self.trajTrgtAvgY - self.initPos[1] 
            self.spzCor = self.trajTrgtAvgZ - self.initPos[2]  
    #
    def check_traj_position(self,*args):
        i = self.mavProgram.seqIdCur
        j = self.mavProgram.trajIdCur
        d = self.trajThreshDist
        #calculate pseudo distance to current trajectory position
        #dx1 = abs(self.vsnx-self.mavProgram.x[i][j])
        #dy1 = abs(self.vsny-self.mavProgram.y[i][j])
        #dz1 = abs(self.vsnz-self.mavProgram.z[i][j])
        
        #calculate pseudo distance to next trajectory position
        
        #variant 1
        #dx2 = abs(self.vsnx- self.mavProgram.x[i][j+3])
        #dy2 = abs(self.vsny- self.mavProgram.y[i][j+3])
        #dz2 = abs(self.vsnz- self.mavProgram.z[i][j+3])
        
        #variant 2
        dx2 = abs(self.vsnx- self.trajTrgtAvgX)
        dy2 = abs(self.vsny- self.trajTrgtAvgY)
        dz2 = abs(self.vsnz- self.trajTrgtAvgZ)

        #if next position is closer, advance position 
        #if self.debug: print 'trajectory distances:',' x:',dx1,dx2,' y:',dy1,dy2,' z:',dz1,dz2
        if self.debug: print 'trajectory distances:',' x:',dx2,' y:',dy2,' z:',dz2
        
        #if (dx2==dx1 or dx2<dx1) and (dy2<dy1 or dy2==dy1) and (dz2==dz1 or dz2<dz1):
        if (dx2<d and dy2<d and dz2<d):
            if self.mavProgram.advance_trajectory(): # True if reached the end of trajectory
                return True
            else:
                self.calc_avg_traj_target()
                self.correct_setpoint()
    #        
        
    def calc_avg_traj_target(self): # functionality for calculating target pursuit
        i = self.mavProgram.seqIdCur
        j = self.mavProgram.trajIdCur
        l = self.mavProgram.trajEndPos[i]
        n = self.trajTrgtAvgN
        xsum,ysum,zsum = (0.0 for i in range(3))
        # get average vector for n-consecutive targets on trajectory
        if j+n+1 < l: # if farther than n points away from the end of trajectory
            for k in range(j,j+n): # add the next n coordinates
                xsum += self.mavProgram.x[i][k]
                ysum += self.mavProgram.y[i][k]
                zsum += self.mavProgram.z[i][k]
            #
        else:
            for k in range(j,l): # add the rest of points on trajectory
                xsum += self.mavProgram.x[i][k]
                ysum += self.mavProgram.y[i][k]
                zsum += self.mavProgram.z[i][k]
            n = l-j # the number of remaining points in trajectory
        #calculate average direction along each axis
        self.trajTrgtAvgX = xsum/max(n,1) 
        self.trajTrgtAvgY = ysum/max(n,1)
        self.trajTrgtAvgZ = zsum/max(n,1)
        
            
        
    #
        
    def correct_vision(self):

        self.vsnxCor    = self.vsnx -self.initPos[0] 
        self.vsnyCor    = self.vsny -self.initPos[1] 
        self.vsnzCor    = self.vsnz - self.initPos[2]
    #    
    
    def print_info(self,*args):
        
        i = self.mavProgram.seqIdCur
        
        lposx = round(self.lposx,2)
        lposy = round(self.lposy,2)
        lposz = round(self.lposz,2)
        spx = round(self.spxCor,2)
        spy = round(self.spyCor,2)
        spz = round(self.spzCor,2)
        xd = round(   abs(   self.vsnxCor - self.spxCor   ),2   )
        yd = round(   abs(   self.vsnyCor - self.spyCor   ),2   )
        zd = round(   abs(   self.vsnzCor - self.spzCor   ),2   )
        vx = round(self.velx,2)
        vy = round(self.vely,2)
        vz = round(self.velz,2)

        rospy.loginfo('Useful data:')
        print 'current velocity       : '+ '  vel X: ' + str(vx)    + '    vel Y: ' + str(vy) + '  vel Z: ' , str(vz)        
        print 'current local position : '+ '  pos X: ' + str(lposx) + '    pos Y: ' + str(lposy) + '  pos Z: ' , str(lposz) 
        print 'corrected setpoint     : '+ '   sp X: ' + str(spx)   + '     sp Y: ' + str(spy)   + '   sp Z: ' , str(spz)
        print 'distance to setpoint   : '+ ' dist X: ' + str(xd)    + '   dist Y: ' + str(yd)    + ' dist Z: ' + str(zd)  
        print ''
    #

    def check_vision_stream(self,*args): # check for valid data stream
        
        #print 'vision data: ',self.vsnx,self.vsny,self.vsnz
        #print 'visionValid: ',self.visionValid   
         
        if self.visionValid and self.vsnxPrev == self.vsnx and self.vsnyPrev == self.vsny and self.vsnzPrev == self.vsnz: # if prev state is valid and data hasn't changed
            self.visionValid = False
            self.visionTimerStart = rospy.get_time()
            
        elif self.visionValid != True and self.vsnxPrev != self.vsnx and self.vsnyPrev != self.vsny and self.vsnzPrev != self.vsnz: # prev state invalid and data has changed
            self.visionValid = True
            rospy.loginfo('Vision data is VALID!')
            self.overrideLed = [None,None,None]
            
        if not self.visionValid: # if vision not valid calculate for how long
            self.noVisionDuration = round( (rospy.get_time() - self.visionTimerStart),2)
            
            if self.noVisionDuration < self.visionTimeotThr: # warn if it's short time
                rospy.logwarn('Vision data has not changed for '+str(self.noVisionDuration)+' seconds')
                
            if self.noVisionDuration > self.visionTimeotThr: # throw fatal error if data has not resumed
                rospy.logfatal('NO VALID VISION DATA for more than '+str(self.noVisionDuration)+' seconds')
                self.overrideLed = [255,0,0]  
                # take action
                
                if not self.mavState.isArmed():
                    # wait for vision data
                    rospy.logfatal('Waiting for VALID vision data...')
                    while not rospy.is_shutdown():
                        if self.vsnxPrev != self.vsnx and self.vsnyPrev != self.vsny and self.vsnzPrev != self.vsnz:
                            return    
                        self.rate.sleep()    
   
                elif self.mavState.isArmed() and self.vsnz < 0.5: # if not too high, simply disarm
                    rospy.logwarn('Altitude is low, disarming...')
                    #self.mavState.doSetMode('MANUAL')
                    self.mavState.doDisarm()
                    self.mavState.wait_for_disarmed()
                    # TODO wait for a few seconds anf if not disarmed, sys.exit()
                    
                elif self.mavState.isArmed() and self.vsnz > 0.5:
                    rospy.logwarn('Vehicle in air, setting altitude control mode...')
                    self.mavState.wait_for_mode('ALTCTL')
                    rospy.logwarn('LAND the vehicle and DISARM!')
                    self.mavState.wait_for_disarmed()
                    # TODO exit the sequence and wait for user input

        self.vsnxPrev = self.vsnx
        self.vsnyPrev = self.vsny
        self.vsnzPrev = self.vsnz
   
    #
    '''
    def start_accelerated_takeoff(self):

        rospy.logwarn('Starting Accelerated TAKEOFF...')
        timerStart = rospy.Time.now()
        
        while not rospy.is_shutdown():
            
            
            k = self.takeoffHeight /  self.spDistZ # the closer the setpoint, the closer to zero
            
            #print 'self.takeoffHeight: ',self.takeoffHeight 
            #print 'self.spDistZ: ',self.spDistZ
            self.desiredVel[2] = (self.takeoffVel[2] / k ) # the closer the setpoint, the lower velocity
            
            #print 'k: ',k
            #print 'self.desiredVel[2]: ',self.desiredVel[2]
            #print 'self.takeoffVel[2]: ',self.takeoffVel[2]
            #self.check_pause_sw2()
            #if self.debug: rospy.loginfo('    DEBUG: '+'spDistZ: '+str(self.spDistZ) +' spDistXY: '+str(self.spDistXY) )
            
            # IF velocity timer not expired
            if not rospy.Time.now() - timerStart > self.takeoffTimerThr: 
                pass
                #rospy.loginfo('Accelerated takeoff...'+' Vertical velocity: '+str( self.velz )) 
            else: 
                rospy.loginfo('Acceleration timed out...')
                break
            #
            
            # IF reached setpoint
            if self.lposz < self.takeoffTarget[2]-self.spDistZthr:
                pass
                #rospy.loginfo('Vertical distance to setpoint: '+str(self.spDistZ) )
            else:    
                rospy.loginfo('Reached target altitude, Slowing down...')
                break
            #
            
            # IF horiz deviation is big
            if self.spDistXY > 1: 
                rospy.logwarn('XY deviation is LARGE: '+str(self.spDistXY)+' Slowing down...' )
                break
            #
            self.rate.sleep()
        
        self.desiredVel = self.zeroVel
        self.velTrigger = False # stop sending velocity
        #self.takeoffTrigger = False # unblock xy setpoint component
            
        return    
    #
    '''
    
    
