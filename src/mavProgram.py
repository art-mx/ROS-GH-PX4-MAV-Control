import rospy
import sys
import time
import math
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler




class MavProgram:
    
    def __init__(self,debug,rate,mavState):    
        
        rospy.Subscriber("/gh/poselist", String, self.program_update)
        
        self.rate = rate
        self.debug = debug
        self.mavState = mavState
        
        self.seqIdCur = 0 #current sequence id in running program
        self.trajIdCur = 0 #current position on a trajectory
        self.trajEndPos = [] #number of points of each trajectory of a program
        self.seqType = []
        self.x = []
        self.y = []
        self.z = []
        self.rot = []
        self.qw = []
        self.qx = []
        self.qy = []
        self.qz = []
        self.r = []
        self.g = []
        self.b = []

        self.programValid = False
    #
    
    def wait_for_program(self):

        if not self.programValid:
            rospy.logwarn('WAITING for valid program...')       
        while not rospy.is_shutdown():
            if self.programValid:
                return True
                
            self.rate.sleep() 
    #   

    def program_update(self,data):
        
        self.seqId = [] 
        if not self.mavState.currentState.armed:
            rospy.logwarn( 'INFO: '+'Received new program, state=disarmed, APPLYING!')
            self.program_string = data.data
        else: 
            rospy.logwarn( 'INFO: '+'Received new program, WRONG STATE, REJECTING!')
            return
        rospy.loginfo('INFO: '+'Received program: '+'\n'+ str(self.program_string))
                
        prog_sequences = self.program_string.split('\r\n')
        if prog_sequences[0] == 'linear_move':
            rospy.loginfo('Identified linear move program!')
            del prog_sequences[0]
            self.parse_linear_move(prog_sequences)
        elif prog_sequences[0] == 'rgb_trajectory':
            rospy.loginfo('Identified rgb trajectory program!')
            del prog_sequences[0]
            self.parse_trajectory(prog_sequences)
    #
            
    
    def parse_trajectory(self,prog_sequences):
        self.programStartPos = int(prog_sequences[0].split('-')[0])
        self.programEndPos = int(prog_sequences[0].split('-')[1])
        self.seqIdCur = self.programStartPos    
       
        rospy.loginfo( 'programStartPos: '+ str(self.programStartPos) )
        rospy.loginfo( 'programEndPos: '+ str(self.programEndPos) )
        
        for i in range (self.programStartPos+1,self.programEndPos+2):
            seq_elements = prog_sequences[i].split(' ') #split each sequence into elements
            self.seqId.append(       int(seq_elements[0])   ) # store number of the sequence
            #if self.debug: print seq_elements
            
            if seq_elements[1] == 'T':
                self.seqType.append('T')
                
                elements = seq_elements[2].split(',') #height + yaw  for takeoff
                
                self.x.append('') #add placeholders
                self.y.append('')
                self.z.append(float (elements[0]))
                                
                yaw =  math.radians(float (elements[1])) 
                x,y,z,w = quaternion_from_euler(0,0,yaw)
                #if self.debug: print 'quat sp: ',w,x,y,z
                self.qw.append ( w )
                self.qx.append ( x )
                self.qy.append ( y )
                self.qz.append ( z )
                self.r.append(    0.0 )
                self.g.append(    0.0 )
                self.b.append(    0.0 )
                self.trajEndPos.append('') # placeholder
            # 
            
            elif seq_elements[1] == 'Mt': #trajectory move
                self.seqType.append('Mt')
                
                traj_elements = seq_elements[2].split(';') #elements of a trajectory curve
                traj_x,traj_y,traj_z,traj_rot,traj_r,traj_g,traj_b = ([] for i in range(7)) 
                traj_qx,traj_qy,traj_qz,traj_qw = ([] for i in range(4))
                
                if self.debug: rospy.loginfo('Parsing trajectory: '+str(self.seqId[-1]) )
                self.trajEndPos.append( len(traj_elements) )
                if self.debug: rospy.loginfo('Number of points in trajectory: '+str(self.trajEndPos[-1]) )
                for i in range (0,self.trajEndPos[-1]): #elements of one trajectory curve
                    elements = traj_elements[i].split(',') #elements of one point on trajectory curve
                    traj_x.append(      float (elements[0]) )
                    traj_y.append(      float (elements[1]) )
                    traj_z.append(      float (elements[2]) )
                                        
                    yaw =  math.radians(float (elements[3])) 
                    x,y,z,w = quaternion_from_euler(0,0,yaw)
                    #if self.debug: print 'quat sp: ',w,x,y,z
                    
                    traj_qx.append(x)
                    traj_qy.append(y)
                    traj_qz.append(z)
                    traj_qw.append(w)

                    traj_r.append(      float (elements[4]) )
                    traj_g.append(      float (elements[5]) )
                    traj_b.append(      float (elements[6]) )
                    
                    if self.debug: print(str(i),elements[0],elements[1],elements[2],elements[3])
                    if self.debug: print(elements[4],elements[5],elements[6])
                #
                # nested lists
                self.x.append(    traj_x     ) 
                self.y.append(    traj_y     )
                self.z.append(    traj_z     )
                self.qw.append ( traj_qw )
                self.qx.append ( traj_qx )
                self.qy.append ( traj_qy )
                self.qz.append ( traj_qz )
                self.r.append(    traj_r     ) 
                self.g.append(    traj_g     )
                self.b.append(    traj_b     )
            #
                
            if seq_elements[1] == 'M':
                self.seqType.append('M')
                
                elements = seq_elements[2].split(',') #xyzh of a setpoint target
                
                self.x.append(       float(elements[0])   )
                self.y.append(       float(elements[1])   )
                self.z.append(       float(elements[2])   )
                                
                yaw =  math.radians(float (elements[3])) 
                x,y,z,w = quaternion_from_euler(0,0,yaw)
                #if self.debug: print 'quat sp: ',w,x,y,z
                self.qw.append ( w )
                self.qx.append ( x )
                self.qy.append ( y )
                self.qz.append ( z )
                #turn off led during move sequence
                self.r.append(    float (elements[4]) )
                self.g.append(    float (elements[5]) )
                self.b.append(    float (elements[6]) )
                self.trajEndPos.append('') # placeholder
            # 
            
            if seq_elements[1] == 'L':
                self.seqType.append('L')
                #fill with placeholders
                self.x.append('')
                self.y.append('')
                self.z.append('')

                self.qw.append ('')
                self.qx.append ('')
                self.qy.append ('')
                self.qz.append ('')
                # turn off led
                self.r.append(0.0)
                self.g.append(0.0)
                self.b.append(0.0)
                self.trajEndPos.append('') # placeholder

        rospy.loginfo('Program sequences: '+str(self.seqId))
        

        # check if the first sequence is takeoff
        if not self.seqType[self.programStartPos] == 'T':
            rospy.logwarn('ERROR!!! First sequence is not takeoff')
            self.programValid = False
            return
        self.programValid = True       
                    
                
        
            
    def parse_linear_move(self,elements):
        self.programStartPos = int(elements[0].split('-')[0])
        self.programEndPos = int(elements[0].split('-')[1])
        self.seqIdCur = self.programStartPos 
        
        rospy.loginfo( 'programStartPos: '+ str(self.programStartPos) )
        rospy.loginfo( 'programEndPos: '+ str(self.programEndPos) )
        
        for i in range (self.programStartPos+1,self.programEndPos+2):
            self.progSeq = elements[i].split(' ')
            if self.debug: print self.progSeq
            self.seqId.append(       int(self.progSeq[0])   )
            self.seqType.append(     str(self.progSeq[1])   )
            self.x.append  (       float(self.progSeq[2]) )
            self.y.append  (       float(self.progSeq[3]) )
            self.z.append  (       float(self.progSeq[4]) )
            self.rot.append(       float(self.progSeq[5]) ) 
            
            yaw =  math.radians(self.rot[-1]) 
            x,y,z,w = quaternion_from_euler(0,0,yaw)
            if self.debug: print 'quat sp: ',w,x,y,z
    
            self.qw.append ( w )
            self.qx.append ( x )
            self.qy.append ( y )
            self.qz.append ( z )
            
            self.r.append(0.0)
            self.g.append(255.0)
            self.b.append(0.0)
            
        rospy.loginfo('INFO: '+'Program sequences: '+str(self.seqId))

        # check if the first sequence is takeoff
        if not self.seqType[self.programStartPos] == 'T':
            rospy.logwarn('ERROR!!! First sequence is not takeoff')
            self.programValid = False
            return
        self.programValid = True
    #except:
        #rospy.logwarn('ERROR!!! FAILED TO LOAD THE PROGRAM!!!')
       # self.programValid = False
       # return


    #    
    
    def next_sequence(self):
        
        if self.seqIdCur < self.programEndPos:
            self.seqIdCur+=1
            i=self.seqIdCur
            if self.debug: print 'seqIdCur: ',self.seqIdCur ,'programEndPos: ',self.programEndPos
            if self.debug: print 'next sequence type: ',self.seqType[self.seqIdCur]
            if self.debug: print 'next setpoint: ', self.x[i],'\n ',self.y[i],'\n ',self.z[i],'\n'
            
        else:
            rospy.logwarn('INFO: '+'Reached the end of program!')
            time.sleep(5)
            self.wait_for_program()
            #rospy.signal_shutdown('Closing program!')
            sys.exit()
            '''check if vehicle in air, warn, set position control mode'''
            '''if vehicle is on ground, disarm'''
    #
    
    def advance_trajectory(self):
        
        if self.trajIdCur+1 < self.trajEndPos[self.seqIdCur]:
            self.trajIdCur+=1
            if self.debug: print 'trajectory progress: ',self.trajIdCur ,' / ',self.trajEndPos[self.seqIdCur]
        else:
            rospy.loginfo('Reached the end of trajectory!')
            self.trajIdCur = 0
            return True
            
    #
    




        
    
            
            
            