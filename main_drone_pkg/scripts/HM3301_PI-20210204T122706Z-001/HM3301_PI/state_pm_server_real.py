#!/usr/bin/env python3
# must run "sudo pigpiod" before starting
import rospy
import SDL_Pi_HM3301
import time
import std_msgs 
from datetime import datetime
from io import open
import traceback
import pigpio
from main_drone_pkg.srv import sertest,sertestResponse
count=0
every_min = 1
mypi = pigpio.pi()
mySDA = 2
mySCL = 3
time_now = datetime.now()
file_PMname = time_now.strftime('//home/ubuntu/ws_drone_main/src/main_drone_pkg/scripts/logpm/%a_%d_%b_%Y_%H_%M_%S.csv')
num_pm = [ 1, 2.5, 10]
topic = ['_Day_','_Time_','PM1', 'PM2.5', 'PM10', 'PMatm1', 'PMatm2.5', 'PMatm10', 'AQI', 'alt', 'lat', 'lon']
textPM =['PM%.1f Standard particulate matter concentration Unit:ug/m3 = %d','PM%.1f Atmospheric environment concentration ,unit:ug/m3 = %d']
hm3301 = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=mySDA, SCL=mySCL, pi=mypi)

def handle_pm(req):
     print("BEGINING RECIVE PM 1.0,2.5,10")
     if req.begin :
          for x in range(5):
               print("LOOP :: {} ".format((x+1)) )
               time_now = datetime.now()
               log_PM = open(file_PMname,'a+',encoding='utf-8')
               #print(file_PMname)
               log_PM.write(time_now.strftime('%a_%d/%b/%Y %H:%M:%S.%f') + ' ')
               myData = hm3301.get_data()
               if (hm3301.checksum() != True):
                  print("Checksum Error!") 
               myAQI = hm3301.get_aqi()
               for i in range(len(myData)):
                 log_PM.write('%d'%myData[i] + ' ')
                 if (i==5):log_PM.write(myAQI + '\n')
               #hm3301.print_data()
               #print ("AQI=", myAQI)
               log_PM.close
               time.sleep(1)
     else :
          print("not_happen........")
     print("FINISH RECIVE PM 1.0,2.5,10\n")
     return sertestResponse(True)

def state_pm_server():
     rospy.init_node('state_pm_server')
     rospy.Service('state_pm',sertest, handle_pm)
     print("Ready to Begin recive PM.")
     rospy.spin()
 
if __name__ == "__main__":
  try:
     time_now = datetime.now()
     for i in range(len(topic)):
              log_PM = open(file_PMname,'a+',encoding='utf-8')
              print ("topic")
              if (i < len(topic)):
                 log_PM.write('%s'%str(topic[i]) + ' ')
              else:
                 log_PM.write('%s'%str(topic[i]) + '\n' )
              log_PM.close
     log_PM = open(file_PMname,'a+',encoding='utf-8')
     log_PM.write('\n')
     log_PM.close
     log_PM = open(file_PMname,'a+',encoding='utf-8')
     log_PM.write('\n')
     log_PM.close
     #pass_time_M = int(time_now.strftime('%M'))%every_min
     #pass_time_S = int(time_now.strftime('%S'))
     state_pm_server()
  except:
    print ("closing hm3301")
    log_PM.close
    print(traceback.format_exc())
    hm3301.close()
