# test HM3301 Laser Dust Sensor

# must run "sudo pigpiod" before starting

import SDL_Pi_HM3301
import time
from datetime import datetime
import traceback
import pigpio
count=0
mypi = pigpio.pi()
mySDA = 2
mySCL = 3
time_now = datetime.now()
log_PM = open(time_now.strftime('log_PM/%a_%d_%b_%Y_%H_%M_%S.txt'),'w',encoding='utf-8')
num_pm = [ 1, 2.5, 10]
textPM =['PM%.1f Standard particulate matter concentration Unit:ug/m3 = %d','PM%.1f Atmospheric environment concentration ,unit:ug/m3 = %d']

hm3301 = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=mySDA, SCL=mySCL, pi=mypi)
time.sleep(0.01)
try:
    print ("START GET PMxx.x")
    while 1: 
            count = count + 1
            print ("START GET C %d",count)
            time_now = datetime.now()
            log_PM.write(time_now.strftime('%a_%d_%b_%Y_%H:%M:%S.%f') + '\n')
            myData = hm3301.get_data()
            for i in range(len(myData)):
                if (i<3):log_PM.write(textPM[0]%(num_pm[i], myData[i]) + '\n')
                if (i>=3):log_PM.write(textPM[1]%(num_pm[i-3],myData[i])+'\n')
                if (i==5):log_PM.write('\n')
                
            if (hm3301.checksum() != True):
                print("Checksum Error!") 
            #myAQI = hm3301.get_aqi()
            #hm3301.print_data()
            #print ("AQI=", myAQI)

            time.sleep(10)
except:
    print ("closing hm3301")
    log_PM.close
    print(traceback.format_exc())
    hm3301.close()
