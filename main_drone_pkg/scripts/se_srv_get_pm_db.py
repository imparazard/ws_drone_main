#!/usr/bin/env python

import rospy
import SDL_Pi_HM3301# must run "sudo pigpiod" before starting
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
hm3301 = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=mySDA, SCL=mySCL, pi=mypi)
file_PMname= str("//home/ubuntu/ws_drone_main/src/main_drone_pkg/scripts/logpm/zerom.txt")
log_PM = open(file_PMname,'w+',encoding='utf-8')
log_PM.write(u" ")
log_PM.close

def handle_pm(req):
     print("BEGINING RECIVE PM 1.0,2.5,10")
     if req.begin :
               log_PM = open(file_PMname,'a+',encoding='utf-8')
               print(file_PMname)
               myData = hm3301.get_data()
               if (hm3301.checksum() != True):
                  print("Checksum Error!") 
               myAQI = hm3301.get_aqi()
               for i in range(3,len(myData)):
                 log_PM.write(u'{}'.format(myData[i]) + ', ')
                 if (i==5):log_PM.write(u'{}'.format(myAQI) + ', ')
               hm3301.print_data()
               print ("AQI=", myAQI)
               log_PM.close
               time.sleep(1)
     else :
          print("DO not thing........")
     print("FINISH RECIVE PM 1.0,2.5,10\n")
     return sertestResponse(True)

def state_pm_server():
     rospy.init_node('state_pm_server_db')
     rospy.Service('/state_pm',sertest, handle_pm)
     print("Ready to Begin recive PM.")
     rospy.spin()
 
if __name__ == "__main__":
  try:
     #pass_time_M = int(time_now.strftime('%M'))%every_min
     #pass_time_S = int(time_now.strftime('%S'))
     state_pm_server()
  except:
    print ("closing hm3301")
    log_PM.close
    print(traceback.format_exc())
    hm3301.close()
