#!/usr/bin/env python
#filename : se_state_leveldrone_indoor_pm.py
import time
import rospy
import smach
import rosservice
import mavros
import smach_ros 
import numpy as np
import math
import io
from smach import State
from mavros_msgs.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import *
from ar_track_alvar_msgs.msg import AlvarMarkers
from main_drone_pkg.srv import sertest,sertestResponse
############################def###########################################
########################class##########################################
class getid:
    def __init__(self,id_in,ratio):
        self.ratio = ratio
        self.dictt = {'Active':0, 'id':id_in, 'x':0 ,'y':0, 'z':0, 'anx':0, 'anw':0 , 'r':self.ratio ,'abs':0}

class PID_xyz:  
    def __init__(self,P = 0.6,I = 0.000000, D = 0.00001): 
        self.pre_error = 0.0
        self.integral = 0.0
        self.my_vel = 0.0
        self.output = 0.0
        self.Kp = P
        self.Ki = I
        self.Kd = D
    def pid_ctl(self,relpose,setpoint = 0):
        #Ku = 2.8
        #Pu = 2.3
        #0.6*Ku
        #2*Kp/Pu
        #Kp*Pu/8
        interationtime = 1.0/10
        self.errors = setpoint - relpose
        self.integral = self.integral + self.errors * interationtime
        if self.integral > 50: self.integral = 50
        if self.integral < -50: self.integral = -50
        #print('integral : %f'%self.integral)
        self.derivertive = (self.errors - self.pre_error)/interationtime
        self.output = (self.Kp * self.errors + self.Ki * self.integral + self.Kd * self.derivertive )
        self.pre_error = self.errors
        #print('output : %f'%self.output)
        if self.output > 1.0 :
            self.output = 1.0
        if self.output < -1.0:
            self.output = -1.0
    def reset(self):
        self.integral = 0
        self.derivertive = 0
        self.pre_error = 0
        self.errors = 0
    
   
class god_data:
    def __init__(self, name = 'anonymous'):
        self.name = name
        rospy.logwarn('|||||||||||||||||||||---->|OBJECT_[{}]_WAS_BORN|<----|||||||||||||||||||||'.format(self.name))
        self.arm_chack = 0
        self.mode_chack = 0
        self.PositionTarget = PositionTarget() #/mavros/setpoint_raw/local  msg
        self.hight_GND = 0
        self.localAR_pose = { 'x': 0, 'y': 0, 'z': 0 } #/mavros/local_position/pose
        self.local_home = { 'x': 0, 'y': 0, 'z': 0 } #itfix
        self.local_rel = { 'x': 0, 'y': 0, 'z': 0 } #local relative home value //start at takeoff
        self.gps_pose = { 'lat': 0, 'lon': 0, 'alt': 0 }
        self.Posez_re_pm = 0
        self.pub_position = 0 #list of position for recive data pm 2.5
        self.rate = 0
        self.compensatex = 0
        self.compensatey = 0
        self.pub_vel = 0
        self.count = 0
        self.IdNum = 0
        self.loopfail = 0
        self.Idcount = 0
        self.connect = True
        self.Reclist = 0
        self.RecCount = 0
        self.sethight = 0
        self.bool_hover = 0
        self.gps_home = {'alt':0, 'lat':0, 'lon':0}
        self.makerid = 0
        self.posem_id12 = { 'x': 0, 'y': 0, 'z': 0 } #maker id12
        self.pub_GB_pose = 0
        self.gps_lat = 0
        self.gps_lon = 0
        self.gps_alt = 0
        self.degreeAXIS = 30
        self.vel_ar_msg = TwistStamped() #velocity
        self.infoAR_id = {'1':getid(1,0.9), '10':getid(10,0.9) ,'12':getid(12,0.9), '80':getid(80,0.23), '20':getid(20,0.9) ,'30':getid(30,0.9),'40':getid(40,0.9)}
        self.pidx = PID_xyz(0.55,0.0,0.1)#(1.0,0.0,0.45)
        self.pidy = PID_xyz(0.55,0.0,0.1)
        self.pidanz = PID_xyz(1.2,0.00,0.004)
        self.pidz = PID_xyz(1.2,0.000,0.002)
                                                                                                                                           
    def f_error_mp(self,fix_a, value, err):
        if ( value <= (fix_a + err) ) & ( value >= (fix_a - err) ):
           return False
        else:
           return True
    def f_compensate(self,msg):
        self.compensatex = msg.x
        self.compensatey = msg.y
    def f_makers_pose(self,msg):
        makerid = msg.markers 
        min = 0
        for i in range(len(makerid)):
                if '%d'%(makerid[i].id) in self.infoAR_id:
                    self.infoAR_id['%d'%(makerid[i].id)].dictt['Active'] = True
                    self.infoAR_id['%d'%(makerid[i].id)].dictt['x'] =  (-self.compensatex + makerid[i].pose.pose.position.y) * 1 * self.infoAR_id['%d'%(makerid[i].id)].dictt['r']
                    self.infoAR_id['%d'%(makerid[i].id)].dictt['y'] =  (-self.compensatey + makerid[i].pose.pose.position.x) * 1 * self.infoAR_id['%d'%(makerid[i].id)].dictt['r']
                    self.infoAR_id['%d'%(makerid[i].id)].dictt['z'] =  makerid[i].pose.pose.position.z * 1 * self.infoAR_id['%d'%(makerid[i].id)].dictt['r']
                    self.infoAR_id['%d'%(makerid[i].id)].dictt['anx'] =  abs(makerid[i].pose.pose.orientation.x) 
                    self.infoAR_id['%d'%(makerid[i].id)].dictt['anw'] =  makerid[i].pose.pose.orientation.x * makerid[i].pose.pose.orientation.y
                    self.infoAR_id['%d'%(makerid[i].id)].dictt['abs'] = ((self.infoAR_id['%d'%(makerid[i].id)].dictt['x']**2)+(self.infoAR_id['%d'%(makerid[i].id)].dictt['y']**2)+(self.infoAR_id['%d'%(makerid[i].id)].dictt['z']**2))**0.5
                    if (i == 0) | (self.infoAR_id['%d'%(makerid[i].id)].dictt['abs'] < min) :
                        min = self.infoAR_id['%d'%(makerid[i].id)].dictt['id']
        

    def f_error_return(self):
        err = 0.2
        self.f_update_target_rel()
        if ((self.f_error_mp(self.PositionTarget.position.z ,self.local_pose['z'], err)) | (self.f_error_mp(self.PositionTarget.position.x , self.local_pose['x'], err)) | (self.f_error_mp(self.PositionTarget.position.y ,self.local_pose['y'], err))):
           return False
        else:
           return True

    def f_arm(self):
            rospy.wait_for_service('/mavros/cmd/arming')
            try:
                armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
                armService(True)
            except rospy.ServiceException as e:
                print ("Service arm call failed: %s" %e)

    def f_offboard(self):
            rospy.wait_for_service('/mavros/set_mode')
            try:
                flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
                isModeChanged = flightModeService(custom_mode='OFFBOARD') #return true or false
            except rospy.ServiceException as e:
                print ("service set_mode call failed: %s OFFBOARD Mode could not be set" %e)
    def f_altctl(self):
            rospy.wait_for_service('/mavros/set_mode')
            try:
                flightModeService = rospy.ServiceProxy('/mavros/set_mode', mavros_msgs.srv.SetMode)
                isModeChanged = flightModeService(custom_mode='ALTCTL') #return true or false
            except rospy.ServiceException as e:
                print ("service set_mode call failed: %s OFFBOARD Mode could not be set" %e)

    def f_hight(self, msg) :
        self.hight_GND = msg.range #recived range from px4flow
        #rospy.logwarn("hight_GND : {} ".format( ADT.hight_GND))

    def f_cheack_state(self, msg):
        self.mode_chack = msg.mode
        self.arm_chack = msg.armed
        self.connect = msg.connected

    def f_update_target_rel(self):
        self.PositionTarget.position.x = self.local_home['x'] + self.local_rel['x']
        self.PositionTarget.position.y = self.local_home['y'] + self.local_rel['y']
        self.PositionTarget.position.z = self.local_home['z'] + self.local_rel['z']

    def f_pub_target_rel(self):
        self.f_update_target_rel()
        self.puynt ("Service set home call failed: %s" %e )

    def f_service_pm(self):
        rospy.wait_for_service('state_pm')
        try:
            Servicetest = rospy.ServiceProxy('state_pm', sertest) 
            resp = Servicetest(True)
            return resp.end
        except rospy.ServiceException as e:
            print ("Service PM2.5 call failed: %s" %e )

    def f_LandMode(self):
        rospy.wait_for_service('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService(altitude = self.gps_home['alt'], latitude = self.gps_home['lat'], longitude = self.gps_home['lon'] , min_pitch = 0, yaw = 0)
        except rospy.ServiceException as e:
            print ("service land call failed: %s. The vehicle cannot land " %e)

    def f_Disarm(self):
        rospy.wait_for_service('/mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('/mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. DISARM "%e)

    def uppm(self):
        rospy.wait_for_service('/state_upload')
        try:
            Servicetest = rospy.ServiceProxy('/state_upload', sertest) 
            resp = Servicetest(True)
            return resp.end
        except rospy.ServiceException as e:
            print ("Service state_upload call failed: %s"%e    )
            rospy.logwarn('--- FIX BY MANUAL ---')
##############################################################################################################################
class c_check_prepair(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['prepare_success','prepare_fail'])
    def execute(self, userdata):
        pm = True
        upload = True
        clearlog = True
        rospy.loginfo('Executing state CHECK_PREPAIR')
        try:
          file_PMname = str("//home/ubuntu/ws_drone_main/src/main_drone_pkg/scripts/logpm/zerom.txt")
          log_PM = io.open(file_PMname,'w+',encoding='utf-8')
          #log_PM.write(" ")
          log_PM.close
          rospy.loginfo("Clear Log PM : [success] ")
        except rosservice.ROSServiceException as e:
          rospy.logwarn("Clear Log PM  : [wrong] ")
          clearlog = False
        try:
          ServiceClass = rosservice.get_service_class_by_name('/state_pm')
          rospy.loginfo("Status Service recive P.M. 2.5 : [online] ")
        except rosservice.ROSServiceException as e:
          rospy.logwarn("Status Service recive P.M. 2.5 : [offline] ")
          pm = False

        try:
          ServiceClass = rosservice.get_service_class_by_name('/state_upload')
          rospy.loginfo("Status Service Upload P.M. 2.5 : [online] ")
        except rosservice.ROSServiceException as e:
          rospy.logwarn("Status Service Upload P.M. 2.5 : [offline] ")
          upload = False
        #check connected to fc
        if(ADT.connect):
            rospy.loginfo("Status connected FC : [online] ")
            connected_fc = True
        else:
            rospy.logwarn("Status connected FC : [offline] ")
            connected_fc = False
        if(ADT.infoAR_id['10'].dictt['Active']):
            rospy.loginfo("Status AR_TRACK_ALVAR : [online] ")
            prepare_ar = True
        else:
            rospy.logwarn("Status AR_TRACK_ALVAR: [offline] ")
            prepare_ar = False
        #check state imu etc.
        rospy.loginfo("-------------------------------------------------")
        if((pm)&(connected_fc)&(prepare_ar)&(clearlog)&(upload)):
          return 'prepare_success'
        else:
          return 'prepare_fail'

class c_offboard(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['offboard_success','offboard_fail'])
    def execute(self, userdata):
        rospy.loginfo('Executing state OFFBOARD')
        ADT.vel_ar_msg.twist.linear.z = 0.5
        ADT.vel_ar_msg.twist.linear.y = 0
        ADT.vel_ar_msg.twist.linear.x = 0
        cout = 0
        while ((not ADT.arm_chack ) | ( ADT.mode_chack != 'OFFBOARD') ) :
            if cout == 50: return 'offboard_fail'
            ADT.f_arm()  
            ADT.f_offboard()
            ADT.pub_vel.publish(ADT.vel_ar_msg)
            ADT.rate.sleep()
            rospy.logwarn("ARM : {} ,MODE : {} ".format( ADT.arm_chack ,ADT.mode_chack ))
            cout += 1
        return 'offboard_success'

class c_track_RP(smach.State):
    global ADT
    def __init__(self):
        smach.State.__init__(self, outcomes=['RP_success','RP_remain','To_GetPm'])
    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_RP')
        IDcount = ADT.Idcount
        ID = ADT.IdNum[IDcount]
        hight = ADT.sethight[ID]
        if(ADT.Idcount < (len(ADT.IdNum)-1)):
          CC = IDcount + 1
        else:
          CC = IDcount - 1
        nextID = ADT.IdNum[CC]
        while True:
                ADT.pidx.pid_ctl(ADT.infoAR_id[ID].dictt['x'] )
                ADT.pidy.pid_ctl(ADT.infoAR_id[ID].dictt['y'] )
                ADT.pidz.pid_ctl(ADT.infoAR_id[ID].dictt['z'], hight )
                ADT.vel_ar_msg.twist.linear.x = ADT.pidx.output
                ADT.vel_ar_msg.twist.linear.y = ADT.pidy.output
                ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
                ADT.pub_vel.publish(ADT.vel_ar_msg)
                print('Follow ID : %s '%(ID))
                print('target X : %.6f m.'%(-ADT.infoAR_id[ID].dictt['x']))
                print('target Y : %.6f m.'%(-ADT.infoAR_id[ID].dictt['y']))
                print('target Z : %.6f m.'%(ADT.infoAR_id[ID].dictt['z']))
                print('ADT.Idcount : %d '%(ADT.Idcount))
                print('ADT.RecCount : %d '%(ADT.RecCount))
                print('len(ADT.IdNum)-1 : %d '%(len(ADT.IdNum)-1))
                if((ADT.infoAR_id[ID].dictt['z'] > (hight-0.5))&(ADT.infoAR_id[ID].dictt['z'] < (hight+0.5))&(abs(ADT.infoAR_id[ID].dictt['x']) < 0.1) & (abs(ADT.infoAR_id[ID].dictt['y']) < 0.1)):
                   if( ADT.IdNum[IDcount] == ADT.Reclist[ADT.RecCount]):
                      return 'To_GetPm'
                   elif((ADT.Idcount >= (len(ADT.IdNum)-1)) & (ADT.infoAR_id[nextID].dictt['Active'])):
                      ADT.Idcount -= 1
                      return 'RP_success'
                   elif((ADT.Idcount < (len(ADT.IdNum)-1)) & (ADT.infoAR_id[nextID].dictt['Active'])):
                      ADT.Idcount += 1
                      return 'RP_remain'
                   else:
                      rospy.logwarn('...NOT Found Next AR Track!')
                ADT.rate.sleep()

class c_recive_pm(smach.State):
    global ADT
    def __init__(self):
        smach.State.__init__(self, outcomes=['recive_success','recive_next'])
    def execute(self, userdata):
            rospy.loginfo('#Executing state RECIVE PM 2.5  ')
            rospy.loginfo("Loading... recive pm2.5  ")
            ADT.f_service_pm()
            ADT.RecCount += 1
            ADT.bool_hover = False
            return 'recive_success'

class c_hover(smach.State):
    global ADT
    def __init__(self):
        smach.State.__init__(self, outcomes=['hover_success'])
    def execute(self, userdata):
          ADT.bool_hover = True
          IDcount = ADT.Idcount
          ID = ADT.IdNum[IDcount]
          hight = ADT.sethight[ID]
          rospy.loginfo('#Executing state HOVER')
          while (ADT.bool_hover):
            print('Hover')
            ADT.pidx.pid_ctl(ADT.infoAR_id[ID].dictt['x'] )
            ADT.pidy.pid_ctl(ADT.infoAR_id[ID].dictt['y'] )
            ADT.pidz.pid_ctl(ADT.infoAR_id[ID].dictt['z'],hight )
            ADT.vel_ar_msg.twist.linear.x = ADT.pidx.output
            ADT.vel_ar_msg.twist.linear.y = ADT.pidy.output
            ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
            ADT.pub_vel.publish(ADT.vel_ar_msg)
            ADT.rate.sleep()
          return 'hover_success'

class c_track_WayBh(smach.State):
    global ADT
    def __init__(self):
        smach.State.__init__(self, outcomes=['WayBH_success','WayBH_remain'])
    def execute(self, userdata):
        rospy.loginfo('Executing state TRACK_RP')
        IDcount = ADT.Idcount
        ID = ADT.IdNum[IDcount]
        if(ADT.Idcount <= 0):
          CC = 0
        else:
          CC = IDcount - 1
        preID = ADT.IdNum[CC]
        while True:
                ADT.pidx.pid_ctl(ADT.infoAR_id[ID].dictt['x'] )
                ADT.pidy.pid_ctl(ADT.infoAR_id[ID].dictt['y'] )
                ADT.pidz.pid_ctl(ADT.infoAR_id[ID].dictt['z'],1 )
                ADT.vel_ar_msg.twist.linear.x = ADT.pidx.output
                ADT.vel_ar_msg.twist.linear.y = ADT.pidy.output
                ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
                ADT.pub_vel.publish(ADT.vel_ar_msg)
                print('target X : %.6f m.'%(-ADT.infoAR_id[ID].dictt['x']))
                print('target Y : %.6f m.'%(-ADT.infoAR_id[ID].dictt['y']))
                print('target Z : %.6f m.'%(ADT.infoAR_id[ID].dictt['z']))
                print('AR now {}'.format(ID))
                print('ADT.Idcount {}'.format(ADT.Idcount))
                if((ADT.infoAR_id[ID].dictt['z'] > 1.0)&(abs(ADT.infoAR_id[ID].dictt['x']) < 0.1) & (abs(ADT.infoAR_id[ID].dictt['y']) < 0.1)):
                   print(' in pricision ')
                   if((ADT.infoAR_id['1'].dictt['Active']) & (ADT.infoAR_id[ADT.IdNum[0]].dictt['Active'])& (abs(ADT.infoAR_id[ADT.IdNum[0]].dictt['x']) < 0.1) & (abs(ADT.infoAR_id[ADT.IdNum[0]].dictt['y']) < 0.1)):
                      return 'WayBH_success'
                   elif((ADT.infoAR_id[preID].dictt['Active']) & (ADT.Idcount > 0)):
                      ADT.Idcount -= 1
                      return 'WayBH_remain'
                ADT.rate.sleep()

class c_track_land(smach.State):
    global ADT
    def __init__(self):
        smach.State.__init__(self, outcomes=['track_center','track_fail','track_fail_landnow'])
    def execute(self, userdata):
        rospy.loginfo('Executing state LANDING')
        ADT.pidz.reset()
        ADT.pidx.reset()
        ADT.pidy.reset()
        pa = 0
        cnum = 0
        last_hight = 0
        confirm = True
        failnum = 0
        ban12 = 0
        XfAR = 0.3
        errXY = 0.1
        ZBeforeland = 0.5 #max 0.8
        IdForLand = '1'
        while True:
            if (ADT.infoAR_id[IdForLand].dictt['Active']): 
                ADT.pidx.pid_ctl(ADT.infoAR_id[IdForLand].dictt['x'], -XfAR)
                ADT.pidy.pid_ctl(ADT.infoAR_id[IdForLand].dictt['y'] )
                ADT.pidanz.pid_ctl(ADT.infoAR_id[IdForLand].dictt['anx'], 1.0)
                print('target X : %.6f m.'%(-ADT.infoAR_id[IdForLand].dictt['x']))
                print('target Y : %.6f m.'%(-ADT.infoAR_id[IdForLand].dictt['y']))
                print('target YAW : %.6f m.'%(ADT.infoAR_id[IdForLand].dictt['anx']))
                print('data Z : %.6f m.'%(ADT.infoAR_id[IdForLand].dictt['z']))
                ADT.vel_ar_msg.twist.linear.x = ADT.pidx.output
                ADT.vel_ar_msg.twist.linear.y = ADT.pidy.output

                if((ADT.f_error_mp(XfAR,abs(ADT.infoAR_id[IdForLand].dictt['x']),errXY)) | (abs(ADT.infoAR_id[IdForLand].dictt['y']) > 0.1) | (ADT.infoAR_id[IdForLand].dictt['anx'] < 0.98 ) ) & (confirm)  :
                    ADT.pidz.pid_ctl(ADT.infoAR_id[IdForLand].dictt['z'],1.0 )
                    ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
                    print('First state landing')

                elif ((ADT.infoAR_id[IdForLand].dictt['z'] > ZBeforeland + 0.2) & (not(ADT.f_error_mp(XfAR,abs(ADT.infoAR_id[IdForLand].dictt['x']),errXY))) & ( abs(ADT.infoAR_id[IdForLand].dictt['y']) < errXY)) :
                    ADT.vel_ar_msg.twist.linear.z = -0.1
                    confirm = False
                    print('**************************more %f ******************************'%(ZBeforeland+ 0.2))

                elif ((ADT.f_error_mp(XfAR,abs(ADT.infoAR_id[IdForLand].dictt['x']),errXY)) | (abs(ADT.infoAR_id[IdForLand].dictt['y']) > errXY) | (ADT.f_error_mp(ZBeforeland,(ADT.infoAR_id[IdForLand].dictt['z']),0.1))) | (ADT.infoAR_id[IdForLand].dictt['anx'] < 0.985):
                    ADT.pidz.pid_ctl(ADT.infoAR_id[IdForLand].dictt['z'],ZBeforeland )
                    ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
                    print('**************************pidz %f ******************************'%ADT.pidz.output)
                    cnum = 0
                    confirm = False
                    print((not(ADT.f_error_mp(XfAR,abs(ADT.infoAR_id[IdForLand].dictt['x']),errXY))))
                    print('**************************more {} ******************************'.format(ZBeforeland))

                elif ( (not(ADT.f_error_mp(XfAR,abs(ADT.infoAR_id[IdForLand].dictt['x']),errXY))) & (abs(ADT.infoAR_id[IdForLand].dictt['y']) <= errXY) & (ADT.infoAR_id[IdForLand].dictt['anx'] >= 0.985) & (not(ADT.f_error_mp(ZBeforeland,(ADT.infoAR_id[IdForLand].dictt['z']),0.1))) ):
                    
                    ADT.pidz.pid_ctl(ADT.infoAR_id[IdForLand].dictt['z'],ZBeforeland )
                    ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
                    confirm = False
                    if cnum >= 25:
                       return 'track_center'#pass#
                    else: cnum += 1
                    print('**************************TRY STATBILIZE %d ******************************'%cnum)
                else : 
                       ADT.pidz.pid_ctl(ADT.infoAR_id[IdForLand].dictt['z'],ZBeforeland )
                       ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
                       print ("#####################################  UNKNOW  ######################################")
                ADT.infoAR_id[IdForLand].dictt['Active'] = False
                
                pa = 1
                failnum = 0
                ban12 = 1
                if(ADT.infoAR_id[IdForLand].dictt['anw'] > 0):
                    ADT.vel_ar_msg.twist.angular.z = -ADT.pidanz.output
                else:
                    ADT.vel_ar_msg.twist.angular.z = ADT.pidanz.output
                print ("FOLLOW....ID_1")
                print (ADT.vel_ar_msg.twist.linear.x)
                print (ADT.vel_ar_msg.twist.linear.y)
                print (ADT.vel_ar_msg.twist.linear.z)
                print (ADT.vel_ar_msg.twist.angular.z)

            elif (ADT.infoAR_id['12'].dictt['Active']) & ban12 == 0:
                ADT.pidx.pid_ctl(ADT.infoAR_id['12'].dictt['x'] )
                ADT.pidy.pid_ctl(ADT.infoAR_id['12'].dictt['y'] )
                ADT.pidz.pid_ctl(ADT.infoAR_id['12'].dictt['z'],1.2 )
                ADT.vel_ar_msg.twist.linear.x = ADT.pidx.output
                ADT.vel_ar_msg.twist.linear.y = ADT.pidy.output
                ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
                ADT.infoAR_id['12'].dictt['Active'] = False
                pa = 1
                failnum = 0

            else:
                print(ADT.hight_GND)
                if (pa == 1):
                    last_hight = ADT.hight_GND
                    pa = 0             
                ADT.vel_ar_msg.twist.linear.x = 0
                ADT.vel_ar_msg.twist.linear.y = 0
                ADT.vel_ar_msg.twist.angular.z = 0
                ADT.pidz.pid_ctl(ADT.hight_GND, last_hight)  
                ADT.vel_ar_msg.twist.linear.z =  ADT.pidz.output           
                print ("NOT FOLLOW 1")
                print (ADT.vel_ar_msg.twist.linear.x)
                print (ADT.vel_ar_msg.twist.linear.y)
                print (ADT.vel_ar_msg.twist.linear.z)
                failnum += 1
                print(failnum)
                if (failnum >= 100):
                    ADT.loopfail += 1
                    if(ADT.loopfail >= 4):return'track_fail_landnow'
                    else:return'track_fail'

            ADT.pub_vel.publish(ADT.vel_ar_msg)
            ADT.rate.sleep()

class c__home_land(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_sleep'])
    def execute(self, userdata):
        rospy.loginfo('Executing state DOWNFAST')
        while True:
                if(ADT.infoAR_id['1'].dictt['Active']):
                   ADT.pidx.pid_ctl(ADT.infoAR_id['1'].dictt['x'] )
                   ADT.pidy.pid_ctl(ADT.infoAR_id['1'].dictt['y'] )
                   ADT.vel_ar_msg.twist.linear.x = ADT.pidx.output
                   ADT.vel_ar_msg.twist.linear.y = ADT.pidy.output
                   ADT.infoAR_id['1'].dictt['Active'] = False
                   print('target X : %.6f m.'%(-ADT.infoAR_id['1'].dictt['x']))
                   print('target Y : %.6f m.'%(-ADT.infoAR_id['1'].dictt['y']))
                   print('target YAW : %.6f m.'%(ADT.infoAR_id['1'].dictt['anx']))
                else:
                   ADT.vel_ar_msg.twist.linear.x = 0
                   ADT.vel_ar_msg.twist.linear.y = 0
                   print ("NOT SEE ID 1")
                ADT.vel_ar_msg.twist.linear.z = -0.5
                ADT.pub_vel.publish(ADT.vel_ar_msg)
                print('data Z : %.6f m.'%(ADT.infoAR_id['1'].dictt['z']))
                print('LANDING..............{} cm'.format(ADT.hight_GND))
                if ADT.hight_GND < 0.3:
                   print('LANDING..............SUCCESS')
                   time.sleep(5)
                   ADT.f_altctl()
                   time.sleep(1)
                   ADT.f_Disarm()
                   return 'go_sleep'
                ADT.rate.sleep()
                
class c_track_off30(smach.State):
    global ADT
    def __init__(self):
        smach.State.__init__(self, outcomes=['track_off30'])
    def execute(self, userdata):
        print('in takeoff')
        TfTakeoff = '1'
        while True:
            if ((ADT.hight_GND < 0.8) & (not(ADT.infoAR_id['1'].dictt['Active']))):
                ADT.pidz.pid_ctl(ADT.hight_GND, 1.2)
                ADT.vel_ar_msg.twist.linear.x = 0.0
                ADT.vel_ar_msg.twist.linear.y = 0.0
                if (ADT.hight_GND < 0.2):
                   ADT.vel_ar_msg.twist.linear.z = 0.4
                elif (ADT.hight_GND < 0.5):
                   ADT.vel_ar_msg.twist.linear.z = 0.2
                else:
                   ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output

            elif (ADT.infoAR_id[TfTakeoff].dictt['Active']):

                if ((ADT.infoAR_id[TfTakeoff].dictt['z'] > 1.1)&(abs(ADT.infoAR_id[TfTakeoff].dictt['x']) < 0.1) & (abs(ADT.infoAR_id[TfTakeoff].dictt['y']) < 0.1)):
                   return 'track_off30' 
                   print('####################### hold #########################')
                #ADT.pidanz.pid_ctl(ADT.infoAR_id['12'].dictt['anx'],1.0)
                ADT.pidx.pid_ctl(ADT.infoAR_id[TfTakeoff].dictt['x'],0)
                ADT.pidy.pid_ctl(ADT.infoAR_id[TfTakeoff].dictt['y'],0)
                print('target X : %.6f m.'%(-ADT.infoAR_id[TfTakeoff].dictt['x']))
                print('target Y : %.6f m.'%(-ADT.infoAR_id[TfTakeoff].dictt['y']))
                print('target Z : %.6f m.'%(ADT.infoAR_id[TfTakeoff].dictt['z']))
                ADT.vel_ar_msg.twist.linear.x = ADT.pidx.output
                ADT.vel_ar_msg.twist.linear.y = ADT.pidy.output
                #if(ADT.infoAR_id['12'].dictt['anw'] > 0):
                  #  ADT.vel_ar_msg.twist.angular.z = -ADT.pidanz.output
               # else:
                 #   ADT.vel_ar_msg.twist.angular.z = ADT.pidanz.output
                if (ADT.infoAR_id[TfTakeoff].dictt['z'] < 0.2):
                   ADT.vel_ar_msg.twist.linear.z = 1.0
                elif (ADT.infoAR_id[TfTakeoff].dictt['z'] < 0.1):
                   ADT.vel_ar_msg.twist.linear.z = 0.5
                else:
                   ADT.pidz.pid_ctl(ADT.infoAR_id[TfTakeoff].dictt['z'], 1.2)
                   ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output

                print (ADT.vel_ar_msg.twist.linear.x)
                print (ADT.vel_ar_msg.twist.linear.y)
                print (ADT.vel_ar_msg.twist.linear.z)
                print ("###############takeoff by ....ID_{}##############".format(TfTakeoff))
                #print('**************************takeoff pidz %f ******************************'%ADT.pidz.output)
            else:
                #ADT.pidz.reset()
                #ADT.pidx.reset()
                #ADT.pidy.reset()
                ADT.pidz.pid_ctl(ADT.hight_GND, 1.2)
                ADT.vel_ar_msg.twist.linear.x = 0.0000
                ADT.vel_ar_msg.twist.linear.y = 0.0000
                ADT.vel_ar_msg.twist.linear.z = ADT.pidz.output
                print ("################NOT FOLLOW {} for takeoff############# ".format(TfTakeoff))
                print (ADT.vel_ar_msg.twist.linear.x)
                print (ADT.vel_ar_msg.twist.linear.y)
                print (ADT.vel_ar_msg.twist.linear.z)
            ADT.infoAR_id[TfTakeoff].dictt['Active'] = False
            ADT.pub_vel.publish(ADT.vel_ar_msg)
            ADT.rate.sleep()

        return 'track_off30'
class c__land_acciden(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['go_home'])
    def execute(self, userdata):
        ADT.f_LandMode()
        print('LANDING.................acciden')
        time.sleep(5)
        ADT.f_Disarm()
        return 'go_home'

class c__upload(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['upload_process'])
    def execute(self, userdata):
        rospy.loginfo('Executing state UPLOAD')
        ADT.uppm()
        return 'upload_process'
##################################################---- main ----############################################################ 

def main():
    rospy.init_node('smach_drone_state_machine',anonymous=True )
    global ADT
    ADT = god_data('ADT')
    sm_top = smach.StateMachine(outcomes=['STAY_HOME'])
    rospy.Subscriber("/mavros/distance_sensor/tf_mini_pub",Range, ADT.f_hight )
    #rospy.Subscriber("/mavros/px4flow/ground_distance",Range, ADT.f_hight )
    rospy.Subscriber("/mavros/state",State, ADT.f_cheack_state)
    rospy.Subscriber("/ar_pose_marker",AlvarMarkers, ADT.f_makers_pose)
    rospy.Subscriber("/mavros/xy_compensate",Vector3, ADT.f_compensate)
    ADT.IdNum = ['12', '20', '30', '40']
    ADT.sethight = {'12':1, '20':0.5, '30':0.7, '40':1}
    ADT.Reclist = ['20', '30','40','N/A']
    ADT.Idcount = 0
    ADT.pub_vel = rospy.Publisher("/mavros/setpoint_velocity/cmd_vel",TwistStamped,queue_size=10)
    ADT.rate = rospy.Rate(10)
    with sm_top:
        smach.StateMachine.add('CHECK_PREPARE', c_check_prepair(),
                               transitions={'prepare_success':'OFFBOARD','prepare_fail':'STAY_HOME'})

        smach.StateMachine.add('OFFBOARD', c_offboard(), 
                               transitions={'offboard_success':'TRACK_OFF30','offboard_fail':'STAY_HOME'})

        smach.StateMachine.add('TRACK_OFF30', c_track_off30(), 
                               transitions={'track_off30':'TRACK_Target_RP'})

        smach.StateMachine.add('TRACK_Target_RP', c_track_RP(), 
                               transitions={'RP_success':'TRACK_Target_WayBH','RP_remain':'TRACK_Target_RP','To_GetPm':'RECIVEnHOVER'})

        sm_con = smach.Concurrence(outcomes=['recive_pass','recive_loop'],
                                   default_outcome='recive_loop',
                                   outcome_map={'recive_pass':
                                       { 'RECIVE_PM':'recive_success',
                                         'HOVER':'hover_success'}})
        # Open the container
        with sm_con:
            # Add states to the container
            smach.Concurrence.add('HOVER', c_hover())
            smach.Concurrence.add('RECIVE_PM', c_recive_pm())

        smach.StateMachine.add('RECIVEnHOVER', sm_con,
                               transitions={'recive_loop':'RECIVEnHOVER',
                                            'recive_pass':'TRACK_Target_RP'})

        smach.StateMachine.add('TRACK_Target_WayBH', c_track_WayBh(), 
                               transitions={'WayBH_success':'TRACK_LAND','WayBH_remain':'TRACK_Target_WayBH'})
                              
        smach.StateMachine.add('TRACK_LAND', c_track_land(), 
                               transitions={'track_center':'HOME_LAND','track_fail':'STAY_HOME','track_fail_landnow':'LAND_ACCIDEN'})

        smach.StateMachine.add('LAND_ACCIDEN', c__land_acciden(), 
                               transitions={'go_home':'STAY_HOME'})

        smach.StateMachine.add('HOME_LAND', c__home_land(), 
                               transitions={'go_sleep':'UPLOAD'})

        smach.StateMachine.add('UPLOAD', c__upload(), 
                               transitions={'upload_process':'STAY_HOME'})
                       
    sis = smach_ros.IntrospectionServer("server_drone", sm_top, "/DRONE_ROOT")
    sis.start()                       
    # Execute SMACH plan
    outcome = sm_top.execute()
    rospy.spin()
    sis.stop()
if __name__ == '__main__':
    main()
    

