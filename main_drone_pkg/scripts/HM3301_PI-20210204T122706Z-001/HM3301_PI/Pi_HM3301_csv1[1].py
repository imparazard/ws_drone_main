# supkitrat edit

# must run "sudo pigpiod" before starting

import SDL_Pi_HM3301
import time

time.sleep(5)

from datetime import datetime
import traceback
import pigpio

count=0
mypi = pigpio.pi()
mySDA = 2
mySCL = 3
time_now = datetime.now()
file_PMname = time_now.strftime('log_PM/%a_%d_%b_%Y_%H_%M_%S.csv')
log_PM = open(file_PMname,'w',encoding='utf-8')
num_pm = [ 1, 2.5, 10]
textPM =['PM%.1f Standard particulate matter concentration Unit:ug/m3 = %d','PM%.1f Atmospheric environment concentration ,unit:ug/m3 = %d']

hm3301 = SDL_Pi_HM3301.SDL_Pi_HM3301(SDA=mySDA, SCL=mySCL, pi=mypi)
time.sleep(0.01)
try:
    print("#################################################")
    print("#######       PMPMPMPM   PMPM     PMPM       ####")
    print("#######       PM    PM   PM PM   PM PM       ####")
    print("#######       PMPMPMPM   PM  PM PM  PM       ####")
    print("#######       PM         PM  PMPM   PM       ####")
    print("#######       PM         PM   PM    PM       ####")
    print("#################################################")
    print ("START GET PMxx.x")
    while 1:
            log_PM = open(file_PMname,'a+',encoding='utf-8')
            count = count + 1
            print ("START GET C %d",count)
            print(file_PMname)
            time_now = datetime.now()
            log_PM.write(time_now.strftime('%a_%d_%b_%Y_%H:%M:%S.%f') + ' ')
            myData = hm3301.get_data()
            #if (i<3):log_PM.write(textPM[0]%(num_pm[i], myData[i]) + '\n')
            if (hm3301.checksum() != True):
                print("Checksum Error!") 
            myAQI = hm3301.get_aqi()
            for i in range(len(myData)):
                log_PM.write('%d'%myData[i] + ' ')
                if (i==5):log_PM.write(myAQI + '\n')
            #hm3301.print_data()
            #print ("AQI=", myAQI)
            log_PM.close
            if count == 50 : quit()
            time.sleep(2)
except:
    print ("closing hm3301")
    log_PM.close
    print(traceback.format_exc())
    hm3301.close()
