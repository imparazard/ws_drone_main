#!/usr/bin/env python3

import rospy# must run "sudo pigpiod" before starting
import time
import std_msgs 
import mysql.connector
from datetime import datetime
from random import randint, getrandbits
from main_drone_pkg.srv import sertest,sertestResponse


def handle_pm(req):
     ##################read data pm ######################################
  zerom = open('//home/ubuntu/ws_drone_main/src/main_drone_pkg/scripts/logpm/zerom.txt','r',encoding='utf-8')
  listzerom = zerom.readlines()
  zerom.close()
  listzerom = listzerom[0].split(",")
  li_pm = listzerom# + list20m + list40m
  print(li_pm)
##########################manual###########################################
  today = datetime.now()
  d1 = today.strftime("%Y-%m-%d")#manual
  time1 = today.strftime("%X")#today.strftime("%H:%M:%S")
  print(today.strftime("%H"))
#print("d1 =", d1)
#^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  mydb = mysql.connector.connect(
    host="192.168.43.227",
    user="root",
    password="toor1234",
    database="productdb"
  )

#,randint(30, 210)

  mycursor = mydb.cursor()
#sql = "UPDATE latlng_st SET aqi = '36' WHERE station = 'st1_buu'"#last aqi
###########state1 update last hour ###########
#sql = "INSERT INTO `st1_buu_hour` ( `day`,`hours`, `pm1`, `pm2.5`, `pm10`, `aqi`, `pm1_20`, `pm2.5_20`, `pm10_20`, `aqi_20`, `pm1_40`, `pm2.5_40`, `pm10_40`, `aqi_40`) VALUES ( '2021-04-10' ,'00:49:58', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}'); ".format(randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210))
#mycursor.execute(sql)
###########state2 get to day from database days#################
  sql = "select * from st1_buu_day ORDER BY `day` DESC"
#sql = "select * from st1_buu_hour WHERE day = '2021-04-10' ORDER BY `id` ASC"
  mycursor.execute(sql)
  myresult = mycursor.fetchall()
  ii = 0
  for x in myresult:
    lastday = str(x[1])
    lasthour = str(x[2])
    break


#sql = "select max(pm10),max(aqi) from st1_buu_hour  WHERE day = '2021-04-10' ORDER BY `id` ASC"

############state3 ##################
  if (lastday == d1):
    print("NOT INSERT DAY")
  #################check more 1 hour ####################
    format = '%H:%M:%S'
    timee =  datetime.strptime(time1, format) - datetime.strptime(lasthour, format)
    timee = str(timee)
    print(timee)
    timeestat =  datetime.strptime('00:10:00', format) < datetime.strptime(timee, format)#this use calculate
  ##########################################################
    if(timeestat):
      sql = "UPDATE latlng_st SET date = '{}',aqi = '{}' ,hour = '{}'WHERE station =     'st1_buu'".format(d1,li_pm[3],time1)#aqi on map
      mycursor.execute(sql)
      sql = "INSERT INTO `st1_buu_hour` ( `day`,`hours`, `pm1`, `pm2_5`, `pm10`, `aqi`, `pm1_20`, `pm2_5_20`, `pm10_20`, `aqi_20`, `pm1_40`, `pm2_5_40`, `pm10_40`, `aqi_40`) VALUES ( '{}' ,'{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}'); ".format(d1,time1,li_pm[0],li_pm[1],li_pm[2],li_pm[3],li_pm[4],li_pm[5],li_pm[6],li_pm[7],li_pm[8],li_pm[9],li_pm[10],li_pm[11])
      mycursor.execute(sql)
    #sql  = "UPDATE st1_buu_day SET aqi = '{}' WHERE day = '{}'".format(li_pm[3],d1)
      sql = "select max(pm1),max(pm2_5),max(pm10),max(aqi),max(pm1_20),max(pm2_5_20),max(pm10_20),max(aqi_20),max(pm1_40),max(pm2_5_40),max(pm10_40),max(aqi_40) from st1_buu_hour  WHERE day = '{}' ORDER BY `id` ASC".format(d1)
    #sql = "select * from st1_buu_hour WHERE day = '2021-04-10' ORDER BY `id` ASC"
      mycursor.execute(sql)
      myresult = mycursor.fetchall()
      ii = 0
      for maxpm in myresult:
        print(maxpm)
        #print(d1)
        break
      sql = "UPDATE st1_buu_day SET day = '{}' , hours = '{}', pm1 = '{}', pm2_5 = '{}', pm10 = '{}', aqi = '{}', pm1_20 = '{}', pm2_5_20 = '{}', pm10_20 = '{}', aqi_20 = '{}', pm1_40 = '{}', pm2_5_40 = '{}', pm10_40 = '{}', aqi_40 = '{}' WHERE day= '{}'".format(d1,time1,maxpm[0],maxpm[1],maxpm[2],maxpm[3],maxpm[4],maxpm[5],maxpm[6],maxpm[7],maxpm[8],maxpm[9],maxpm[10],maxpm[11],d1)
      mycursor.execute(sql)

      print("but INSERT HOUR and UPDATE DAY")
    else:
      print("and NOT INSERT HOUR")
  else:
    sql = "UPDATE latlng_st SET date = '{}', aqi = '{}',hour = '{}' WHERE station = 'st1_buu'".format(d1,li_pm[3],time1)#aqi on map
    mycursor.execute(sql)
    sql = "INSERT INTO `st1_buu_day` ( `day`,`hours`, `pm1`, `pm2_5`, `pm10`, `aqi`, `pm1_20`, `pm2_5_20`, `pm10_20`, `aqi_20`, `pm1_40`, `pm2_5_40`, `pm10_40`, `aqi_40`) VALUES ( '{}' ,'{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}'); ".format(d1,time1,li_pm[0],li_pm[1],li_pm[2],li_pm[3],li_pm[4],li_pm[5],li_pm[6],li_pm[7],li_pm[8],li_pm[9],li_pm[10],li_pm[11])
  #sql = "INSERT INTO `st1_buu_day` ( `day`,`hours`, `pm1`, `pm2_5`, `pm10`, `aqi`, `pm1_20`, `pm2_5_20`, `pm10_20`, `aqi_20`, `pm1_40`, `pm2_5_40`, `pm10_40`, `aqi_40`) VALUES ( '{}' ,'{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}'); ".format(d1,time1,randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210),randint(30, 210))
    mycursor.execute(sql)
    sql = "INSERT INTO `st1_buu_hour` ( `day`,`hours`, `pm1`, `pm2_5`, `pm10`, `aqi`, `pm1_20`, `pm2_5_20`, `pm10_20`, `aqi_20`, `pm1_40`, `pm2_5_40`, `pm10_40`, `aqi_40`) VALUES ( '{}' ,'{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}', '{}'); ".format(d1,time1,li_pm[0],li_pm[1],li_pm[2],li_pm[3],li_pm[4],li_pm[5],li_pm[6],li_pm[7],li_pm[8],li_pm[9],li_pm[10],li_pm[11])
    mycursor.execute(sql)
    print("INSERT HOUR and INSERT DAY")


  mydb.commit()
  print("Finish Upload..")
  return sertestResponse(True)

def state_pm_server():
     rospy.init_node('state_upload_server')
     rospy.Service('/state_upload',sertest, handle_pm)
     print("Ready to upload PM. to phpserver")
     rospy.spin()
 
if __name__ == "__main__":
  try:
     #pass_time_M = int(time_now.strftime('%M'))%every_min
     #pass_time_S = int(time_now.strftime('%S'))
     state_pm_server()
  except:
    print ("closing upload")
