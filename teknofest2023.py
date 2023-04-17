#!/usr/bin/env python
# ROS python API

from google.cloud.firestore import GeoPoint
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from geometry_msgs.msg import *
from sensor_msgs.msg import NavSatFix
from threading import Thread
from firebase_admin import credentials
from firebase_admin import firestore

import firebase_admin as firebase
import pyrebase
import cv2
import numpy as np
import rospy
import os
import time
import random2 as random

liste = []
for i in range(1,1000):
    i = str(i)
    liste.append(i+'.avi')
a = random.choice(liste)

config = {
  "apiKey": "AIzaSyB2GMMbPd952xFNQZKRkwPjwQsU-YxT0rM",
  "authDomain": "tay-tulpar.firebaseapp.com",
  "projectId": "tay-tulpar",
  "storageBucket": "tay-tulpar.appspot.com",
  "messagingSenderId": "477071888704",
  "appId": "1:477071888704:web:14e872b341b8d1600daaa5",
  "measurementId": "G-DLRZ7D5BNV",
  "serviceAccount": "/home/tulpar/catkin_ws/src/tulpar/src/scripts/database.json",
  "databaseURL":"https://tay-tulpar-default-rtdb.europe-west1.firebasedatabase.app/"
}

firebase_storage = pyrebase.initialize_app(config)
storage = firebase_storage.storage()

#json dosyasinin konumu
JSON_PATH = "/home/tulpar/catkin_ws/src/tulpar/src/scripts/database.json"
cred = credentials.Certificate(JSON_PATH)
firebase.initialize_app(cred)
database = firestore.client()
database.collection("arananlar").document("aranan").update({"bulundu":False})

global first_latitude
global first_longitude
global localx,localy,localz
global globalposepub
color_found = False
rel_latitude=0
rel_longitude=0
rel_altitude=0
amsl=0

class CallBack:
    def amsl_callback(self, message):
        global amsl
        global rel_altitude
        amsl=float ("{0:.1f}".format (message.amsl))
        rel_altitude=float ("{0:.1f}".format (message.relative))


    def globalpose_callback(self, pose_info):
        global rel_latitude
        global rel_longitude
        global rel_altitude
        

        rel_latitude=pose_info.latitude
        rel_longitude=pose_info.longitude
        rel_altitude=pose_info.altitude

        database.collection("arananlar").document("aranan").update({"location":GeoPoint(rel_latitude, rel_longitude)})
        

    def localpose_callback(self, pose_info):
        global localx,localy,localz
        localx=pose_info.pose.position.x
        localy=pose_info.pose.position.y
        localz=pose_info.pose.position.z

    def status_callback(self, status):
        print("Drone's mode = "+status.mode)
        rospy.sleep(1)
        #rospy.sleep(80)
        #global color_found denemek icin yapildi
        #color_found=True

def Motion(desired_latitude,desired_longitude,altitude):
    global rel_latitude
    global rel_longitude
    global localz
    print("Relative latitude ",rel_latitude," Relative longitude ",rel_longitude,"Relative amsl ",amsl)
    print("Desired locations = ",desired_latitude,desired_longitude)
    print("Time to move")
    global globalposepub
    global amsl
    cnt=controller()
    cnt.information_pub.altitude=amsl +altitude
    cnt.information_pub.latitude=desired_latitude
    cnt.information_pub.longitude=desired_longitude
    rate=rospy.Rate(20)
    while not rospy.is_shutdown ():
        rate.sleep()
        globalposepub.publish(cnt.information_pub)
        rel_latitude=float ("{0:.6f}".format (rel_latitude))
        rel_longitude=float ("{0:.6f}".format (rel_longitude))
        if (abs(desired_latitude-rel_latitude)<0.000002) and (abs(desired_longitude-rel_longitude)<0.000002)and (amsl - 0.4) < cnt.information_pub.altitude < (amsl + 0.4):
            print("Drone has arrived")
            break


class controller:
    def __init__(self):
        self.information_pub=GlobalPositionTarget()
        self.information_pub.coordinate_frame=6
        self.information_pub.type_mask=int ('010111111000', 2) #Plane used to arise so We have changed the type mask
        self.information_pub.latitude=0
        self.information_pub.longitude=0
        self.information_pub.altitude=0

class fcumodes:
    def setarm(self):
        rospy.wait_for_service("mavros/cmd/arming")
        try:
            print("Setting arm")
            set_arm=rospy.ServiceProxy("mavros/cmd/arming",CommandBool)
            set_arm(True)
        except rospy.ServiceException as e:
            print("Service arming call failed: %s" % e)
            pass
    def setDisarm(self):
        rospy.wait_for_service('mavros/cmd/arming')
        try:
            armService = rospy.ServiceProxy('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService(False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s"%e)
            pass
    def auto_rtl(self):
        rospy.wait_for_service('/mavros/set_mode')
        try:
            rtl_service=rospy.ServiceProxy('/mavros/set_mode',SetMode)
            response=rtl_service(custom_mode="AUTO.RTL")
            print("return back to home")
            return response.mode_sent
        except rospy.ServiceException as e:
            print("Service takeoff call failed: %s" %e)
    def take_off(self):
        global rel_latitude, rel_longitude
        global first_latitude
        global first_longitude
        rospy.wait_for_service('mavros/cmd/takeoff')
        try:
            first_latitude=rel_latitude
            first_longitude=rel_longitude
            print("First position = ",format(rel_latitude),format(rel_longitude))
            print('Starting the take_off')
            taking_off=rospy.ServiceProxy('mavros/cmd/takeoff',CommandTOL)
            response =taking_off(min_pitch=0,yaw=0,latitude=rel_latitude,longitude=rel_longitude,altitude=15)
            return response
        except rospy.ServiceException as e: 
            print ("Service takeoff call failed: %s" %e)
    def offboard_mode(self):
        global globalposepub
        rospy.wait_for_service('/mavros/set_mode')
        cnt=controller()
        rate = rospy.Rate(5.0)
        k=0
        while k<10:
            globalposepub.publish(cnt.information_pub)
            rate.sleep()
            k=k+1
            rospy.wait_for_service('/mavros/set_mode')
        try:
            offboard=rospy.ServiceProxy('/mavros/set_mode',SetMode)
            response=offboard(custom_mode="OFFBOARD")
            print("Offboard mode is activated")
            return response.mode_sent
        except rospy.ServiceException as e:
            print("Flight mode has not connected successfuly. The error code = %s"%e)
    def loiter_mode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            holding=rospy.ServiceProxy('mavros/set_mode',SetMode)
            response=holding(custom_mode='AUTO.LOITER')
            return response.mode_sent
        except rospy.ServiceException as e:
            print("Loiter modu bile yapamadik = %s"%e)
    def auto_landmode(self):
        rospy.wait_for_service('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy('mavros/set_mode', SetMode)
            flightModeService(custom_mode='AUTO.LAND')
        except rospy.ServiceException as e:
               print ("service set_mode call failed: %s. Autoland Mode could not be set."%e)
    def setLandMode(self):
        global pos_mode
        pos_mode = False
        rospy.wait_for_service ('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy ('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService (altitude=0)
        except rospy.ServiceException as e:
            print ("service land call failed: %s. The vehicle cannot land " % e)


class Move:
    localposepub=rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=1)
    information_pub=Twist()
    vel_msg = PositionTarget()
    velocity_pub =rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    def MoveX(self, length):
        global localx
        global color_found
        old_x=localx
        self.vel_msg.header.stamp = rospy.get_rostime()
        self.vel_msg.header.frame_id ="local_ned"
        self.vel_msg.coordinate_frame =8
        self.vel_msg.type_mask = int('011111000111', 2)
        if(length>0):self.vel_msg.velocity.y=1.5
        else: self.vel_msg.velocity.y=-1.5
        rate=rospy.Rate(20)
        while (abs(localx-old_x)<abs(length)):
            if color_found == True:
                self.vel_msg.velocity.y=0.0
                self.velocity_pub.publish(self.vel_msg)
                break
            rate.sleep()
            self.velocity_pub.publish(self.vel_msg)
        print("Drone has just arrived first x = ",old_x," now x = ",localx)
        self.vel_msg.velocity.y=0.0
        self.velocity_pub.publish(self.vel_msg)


    def MoveY(self, length):
        global localy
        global color_found
        old_y=localy
        self.vel_msg.header.stamp = rospy.get_rostime()
        self.vel_msg.header.frame_id ="local_ned"
        self.vel_msg.coordinate_frame =8
        self.vel_msg.type_mask = int('011111000111', 2)
        if(length>0):self.vel_msg.velocity.x=1.5
        else: self.vel_msg.velocity.x=-1.5
        rate=rospy.Rate(20)
        while (abs(localy-old_y)<abs(length)):
            if color_found == True:
                self.vel_msg.velocity.x=0.0
                self.velocity_pub.publish(self.vel_msg)
                break
            rate.sleep()
            self.velocity_pub.publish(self.vel_msg)
        print("Drone has just arrived first y = ",old_y," now y = ",localy)
        self.vel_msg.velocity.x=0.0
        self.velocity_pub.publish(self.vel_msg)


    def MoveZ(self, length):
        global localz
        old_z=localz
        global color_found
        rate=rospy.Rate(20)
        if(length>localz):
            self.information_pub.linear.z=1
            while(length>localz):
                rate.sleep()
                self.localposepub.publish(self.information_pub)
        else: 
            self.information_pub.linear.z=-1
            while(length<localz):
                rate.sleep()
                self.localposepub.publish(self.information_pub)
        print("Drone has just arrived first z = ",old_z," now z = ",localz)
        self.information_pub.linear.z=0.0
        self.localposepub.publish(self.information_pub)

    def MoveCenter(self):
        global konum, rel_altitude,color_found, rel_latitude, rel_longitude
        modes = fcumodes()
        rate = rospy.Rate(30)
        if color_found == True:
            self.vel_msg.header.stamp = rospy.get_rostime()
            self.vel_msg.header.frame_id ="local_ned"
            self.vel_msg.coordinate_frame =8
            self.vel_msg.type_mask = int('011111000111', 2)
            self.vel_msg.velocity.z = 0 
            while True:
                if konum == 0:
                    self.vel_msg.velocity.x = 0 
                    self.vel_msg.velocity.y = 0 
                    self.velocity_pub.publish (self.vel_msg) 
                    rate.sleep() 

                    cv2.imwrite("/home/tulpar/Desktop/aa.png", img)
                    storage.child("images/aa.png").put("/home/tulpar/Desktop/aa.png")

                    modes.loiter_mode()
                    rospy.sleep(7)
                    modes.offboard_mode()
        
                    self.MoveZ(6)
                    modes.loiter_mode()
                    rospy.sleep(5)
                    #os.system("python3 pca.py")
                    modes.offboard_mode()
                    self.MoveZ(10)
                    rospy.sleep(5)
                    break

                else:
                    if konum ==1: 
                        self.vel_msg.velocity.x = 0.2 
                        self.vel_msg.velocity.y = 0.2 
                        self.velocity_pub.publish (self.vel_msg) 
                        rate.sleep() 


                    elif konum ==2: 
                        self.vel_msg.velocity.x = 0.15 
                        self.vel_msg.velocity.y = 0.15 
                        self.velocity_pub.publish (self.vel_msg) 
                        rate.sleep() 

                    elif konum ==3: 
                        self.vel_msg.velocity.x = 0.15 
                        self.vel_msg.velocity.y = -0.15 
                        self.velocity_pub.publish (self.vel_msg) 
                        rate.sleep() 


                    elif konum ==4: 
                        self.vel_msg.velocity.x = 0.2
                        self.vel_msg.velocity.y = -0.2 
                        self.velocity_pub.publish (self.vel_msg) 
                        rate.sleep() 

                    elif konum ==5: 
                        self.vel_msg.velocity.x = -0.2 
                        self.vel_msg.velocity.y = 0.2 
                        self.velocity_pub.publish (self.vel_msg) 
                        rate.sleep() 

                    elif konum ==6: 
                        self.vel_msg.velocity.x = -0.15 
                        self.vel_msg.velocity.y = 0.15 
                        self.velocity_pub.publish (self.vel_msg) 
                        rate.sleep() 

                    elif konum ==7: 
                        self.vel_msg.velocity.x = -0.15 
                        self.vel_msg.velocity.y = -0.15 
                        self.velocity_pub.publish (self.vel_msg) 
                        rate.sleep() 

                    elif konum ==8: 
                        self.vel_msg.velocity.x = -0.2
                        self.vel_msg.velocity.y = -0.2 
                        self.velocity_pub.publish (self.vel_msg) 
                        rate.sleep() 
                print('konum = {} '.format(konum))
                    




def image_callback():
    global konum, color_found
    rate = rospy.Rate (20)

    lower_red1 = np.array ([0, 120, 70])
    upper_red1 = np.array ([10, 255, 255])

    # Range for upper range
    lower_red2 = np.array ([170, 120, 70])
    upper_red2 = np.array ([180, 255, 255])

    fourcc = cv2.VideoWriter_fourcc (*'XVID')
    out = cv2.VideoWriter (a, fourcc, 15.0, (1024, 768))

    camSet = 'nvarguscamerasrc !  video/x-raw(memory:NVMM), width=3264, height=2464, ' \
             'format=NV12, framerate=21/1 ! nvvidconv flip-method=' + "2" + \
             ' ! video/x-raw, width=' + "1024" + ', height=' + "768" + \
             ', format=BGRx ! videoconvert ! video/x-raw, format=BGR ! appsink'
    
    cap = cv2.VideoCapture (camSet)
    time.sleep (2.0)

    while not rospy.is_shutdown():
        _, img = cap.read()
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        
        #mask the image (get what you want from image)
        #img_hsv because mask can be applied to hsv
        #mask = cv2.inRange(img_hsv, lower, upper)
        mask1 = cv2.inRange (img_hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange (img_hsv, lower_red2, upper_red2)
        mask = mask1 + mask2

        
        # Remove unnecessary noise from mask
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, (7,7))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, (7,7))

        contours = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        #out.write (img)

        # Determine if one of them exists
        if len(contours)>0:
            c = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(c)
            centerx = int(x)
            centery = int(y)
            rate.sleep()

            if radius>=25:
                #print("COLOR FOUND CENTERING IT")
                color_found = True

                database.collection("arananlar").document("aranan").update({"bulundu_loc":GeoPoint(rel_latitude, rel_longitude), "bulundu":True})
                rate.sleep ()
                    
            
                if (centerx < 256 and centery < 384): konum = 1

                elif (centerx > 256 and centerx < 512 and centery < 384): konum = 2
 
                elif (centerx > 512 and centerx < 768 and centery < 384): konum = 3

                elif (centerx > 768 and centery < 384): konum = 4

                elif (centerx < 256 and centery > 384): konum = 5 

                elif (centerx < 512 and centerx > 256 and centery > 384): konum = 6

                elif (centerx > 512 and centerx < 768 and centery > 384): konum = 7

                elif (centerx > 768 and centery > 384): konum = 8

                elif (centerx > 496 and centerx < 528 and centery < 400 and centery > 368): konum = 0


def mission():
    global globalposepub
    global first_latitude
    global first_longitude
    modes=fcumodes()
    callback = CallBack()
    move = Move()

    rospy.init_node('the_node',anonymous=True)
    rospy.Subscriber('mavros/global_position/raw/fix',NavSatFix,callback.globalpose_callback)
    rospy.Subscriber('mavros/state',State,callback.status_callback)
    rospy.Subscriber('mavros/local_position/pose',PoseStamped,callback.localpose_callback)
    rospy.Subscriber('mavros/altitude',Altitude,callback.amsl_callback)
    globalposepub=rospy.Publisher ('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
    #rospy.sleep(2)

    #start camera as thread (Arka planda baslat)
    Thread(target=image_callback).start()
    modes.setarm()
    rospy.sleep(5)
    #modes.setDisarm()
    modes.take_off()
    #while(localz<9.0):1
    rospy.sleep(10)
    modes.offboard_mode()
    print("Offboard mode activated")
    """Motion(40.993631, 28.724096,0)
    print("Going to waypoint")
    move.MoveCenter()
    print("Moving center")"""
    move.MoveX(6)
    move.MoveY(6)
    move.MoveX(-6)
    move.MoveY(-6)
    """movex = -10
    for i in range(2):
        move.MoveX(movex)
        move.MoveCenter()
        move.MoveY(6)
        move.MoveCenter()
        movex = -movex
    move.MoveX(movex)
    move.MoveCenter()"""

    Motion(first_latitude,first_longitude,0)
    print("Going to first location")
    modes.auto_landmode()
    #print("NO COLOR DETECTED LANDING")
    #rospy.sleep(20)"""
 

if __name__=="__main__":
    try: mission()
    except rospy.ROSInterruptException:
        pass


