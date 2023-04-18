#!/usr/bin/env python
# -*- coding: utf-8 -*-
# ROS python API

import rospy, mavros, math, time
# 3D point & Stamped Pose msgs
from geometry_msgs.msg import Point, PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import NavSatFix, Image
# import all mavros messages and services
from mavros_msgs.msg import *
from mavros_msgs.srv import *
from std_msgs.msg import String, Float64, Int64
from decimal import *
from cv_bridge import CvBridge, CvBridgeError
from math import radians, cos, sin, asin, sqrt
from firebase_admin import credentials
from firebase_admin import firestore
from google.cloud.firestore import GeoPoint
import firebase_admin as firebase
from tulpar.msg import camera
#import Jetson.GPIO as GPIO
global spPub, spGlobPub


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



# Message publisher for local velocity
velocityPub = rospy.Publisher ('mavros/setpoint_raw/local', PositionTarget, queue_size=10)
msg1 = PositionTarget ()
zVelocityPub = rospy.Publisher ('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
msg2 = Twist ()

# Current Position
latitude = 0
longitude = 0
altitude = 0
altitude1 = 0
localX = 0
localY = 0
amsl = 0
previous_longitude=0 
previous_latitude=0
first_latitude=0
first_longitude=0

outputPin = 18  #Water pump PIN
color_found=False
#CV2 Bridge
bridge = CvBridge ()
cv_image = ""

#Target Position
redLatitude = 0
redLongitude = 0
redLatitude2 = 0
redLongitude2 = 0
preRadius = 0
konum = 100
farkx = 0
farky = 0

def altitudeCallback(data):
    global altitude1
    altitude1 = data.relative


mavros.set_namespace ()  # for global callback
def globalPositionCallback(globalPositionCallback):
    global latitude, longitude, altitude
    latitude = globalPositionCallback.latitude
    longitude = globalPositionCallback.longitude
    altitude = globalPositionCallback.altitude
    database.collection("arananlar").document("aranan").update({"location":GeoPoint(latitude, longitude)})


def amslcallback(data):
    global amsl
    amsl = float ("{0:.1f}".format (data.amsl))


def localPositionCallback(localPositionCallback):
    global localX, localY, local_w
    #global color_found
    localX = localPositionCallback.pose.position.x
    localY = localPositionCallback.pose.position.y
    #rospy.sleep(30)
    #color_found=True


def globalPositionPublish(wp_lat, wp_long, wp_alt):
    cnt = Controller ()
    global altitude, latitude, longitude, spGlobPub, amsl
    cnt.sp_glob.latitude = wp_lat
    cnt.sp_glob.longitude = wp_long
    cnt.sp_glob.altitude = amsl + wp_alt
    rate = rospy.Rate (20.0)

    while not rospy.is_shutdown ():

        rate.sleep ()
        spGlobPub.publish (cnt.sp_glob)
        latitude = float ("{0:.6f}".format (latitude))
        longitude = float ("{0:.6f}".format (longitude))

        if (latitude - 0.000002) < wp_lat < (latitude + 0.000002) and (longitude - 0.000002) < wp_long < (
                longitude + 0.000002) and (amsl - 0.4) < cnt.sp_glob.altitude < (amsl + 0.4):
            print ("************ ARRIVED ************")
            break


def haversine():
    global previous_longitude, previous_latitude, longitude, latitude
    lon1, lat1, lon2, lat2 = previous_longitude, previous_latitude, longitude, latitude
    lon1, lat1, lon2, lat2 = map(radians, [lon1, lat1, lon2, lat2])
    dlon = abs(lon2 - lon1)
    dlat = abs(lat2 - lat1)
    a = sin(dlat / 2) ** 2 + cos(lat1) * cos(lat2) * sin(dlon / 2) ** 2
    c = 2 * asin(sqrt(a))
    r = 6371

    return int (c * r * 1000)


def image_callback(radius):
    global redLatitude, redLongitude, latitude, longitude, preRadius
    global color_found
    preRadius = radius
    redLatitude = float ("{0:.6f}".format (latitude))
    redLongitude = float ("{0:.6f}".format (longitude))
    print ("************* RENK ALGILANDI *************", radius, redLatitude, redLongitude)
    database.collection("arananlar").document("aranan").update({"bulundu_loc":GeoPoint(latitude, longitude), "bulundu":True})
    color_found=True
    rate = rospy.Rate (20)
    rate.sleep ()





def cam_konum_callback(data):
    global konum, farkx, farky
    konum = int (data.bolge)
    farkx = int (data.farkx)
    farky = int (data.farky)
    rate = rospy.Rate (20)
    rate.sleep ()
    print ("KONUM=", konum,"FARKX=", farkx,"FARKY=", farky )


# Flight modes class
class fcuModes:
    def __init__(self):
        pass

    def setTakeoff(self):
        global longitude, latitude, first_longitude,first_latitude
        first_latitude=latitude
        first_longitude=longitude
        rospy.wait_for_service ('mavros/cmd/takeoff')
        try:
            takeoffService = rospy.ServiceProxy (
                '/mavros/cmd/takeoff', mavros_msgs.srv.CommandTOL)
            takeoffService (altitude=0, latitude=latitude,
                            longitude=longitude, min_pitch=0, yaw=0)
        except rospy.ServiceException as e:
            print ("Service takeoff call failed: %s" % e)

    def setArm(self):
        rospy.wait_for_service ('mavros/cmd/arming')
        try:
            print ("Waiting for arming...")
            armService = rospy.ServiceProxy ('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService (True)
        except rospy.ServiceException as e:
            print ("Service arming call failed: %s" % e)

    def setDisarm(self):
        rospy.wait_for_service ('mavros/cmd/arming')
        try:
            print ("Waiting for disarming...")
            armService = rospy.ServiceProxy ('mavros/cmd/arming', mavros_msgs.srv.CommandBool)
            armService (False)
        except rospy.ServiceException as e:
            print ("Service disarming call failed: %s" % e)

    def setStabilizedMode(self):
        rospy.wait_for_service ('mavros/set_mode')
        try:
            print ("It's stabilazed mode!")
            flightModeService = rospy.ServiceProxy ('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService (custom_mode='STABILIZED')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Stabilized Mode could not be set." % e)

    def setOffboardMode(self):
        global spGlobPub
        rospy.wait_for_service ('/mavros/set_mode')
        cnt = Controller ()
        rate = rospy.Rate (5.0)
        k = 0
        while k < 12:
            spGlobPub.publish (cnt.sp_glob)
            rate.sleep ()
            k = k + 1
            rospy.wait_for_service ('/mavros/set_mode')

        try:
            flightModeService = rospy.ServiceProxy ('/mavros/set_mode', mavros_msgs.srv.SetMode)
            response = flightModeService (custom_mode='OFFBOARD')
            return response.mode_sent
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Offboard Mode could not be set." % e)
            return False

    def setAltitudeMode(self):
        rospy.wait_for_service ('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy ('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService (custom_mode='ALTCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Altitude Mode could not be set." % e)

    def setLoiterMode(self):
        rospy.wait_for_service ('/mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy ('/mavros/set_mode', mavros_msgs.srv.SetMode)
            isModeChanged = flightModeService (custom_mode='AUTO.LOITER')  # return true or false
        except rospy.ServiceException as e:
            print (
                "service set_mode call failed: %s. AUTO.LOITER Mode could not be set. Check that GPS is enabled %s" % e)

    def setPositionMode(self):
        rospy.wait_for_service ('mavros/set_mode')
        try:
            flightModeService = rospy.ServiceProxy ('mavros/set_mode', mavros_msgs.srv.SetMode)
            flightModeService (custom_mode='POSCTL')
        except rospy.ServiceException as e:
            print ("service set_mode call failed: %s. Position Mode could not be set." % e)

    def setLandMode(self):
        global pos_mode
        pos_mode = False
        rospy.wait_for_service ('/mavros/cmd/land')
        try:
            landService = rospy.ServiceProxy ('/mavros/cmd/land', mavros_msgs.srv.CommandTOL)
            isLanding = landService (altitude=0)
        except rospy.ServiceException as e:
            print ("service land call failed: %s. The vehicle cannot land " % e)



def pose(x, y, z):
        local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size = 10 )

	rate = rospy.Rate(20)
	pose = PoseStamped() #assigning posestamped data to a variable ... posestamped contains position and angular values 

	pose.pose.position.x = x
	pose.pose.position.y = y
	pose.pose.position.z = z #making pose.pose.position.z = 2 so that it can fly to 2metres


	for i in range(100): #we need to send setpoints to drone to set it into OFFBOARD 
		local_pos_pub.publish(pose)
    	rate.sleep()

class Controller:

    # initialization method
    def __init__(self):
        global localX, localY, local_w
        # Drone state
        self.state = State ()  # using that msg for send few setpoint messages, then activate OFFBOARD mode, to take effect
        # Instantiate a setpoints message
        self.sp_glob = GlobalPositionTarget ()
        self.sp_glob.type_mask = int ('010111111000', 2)
        self.sp_glob.coordinate_frame = 6  # FRAME_GLOBAL_INT
        self.sp_glob.latitude = 0
        self.sp_glob.longitude = 0
        self.sp_glob.altitude = 0
        self.level = PoseStamped ()
        self.sp = PositionTarget ()

        self.local_position_publisher = rospy.Publisher (mavros.get_topic ('setpoint_position', 'local'), PoseStamped,
                                                         queue_size=10)
        # set the flag to use position setpoints and yaw angle
        # self.wp.position.z = self.ALT_SP
        self.local_pos = Point (0, 0, 0)
        self.spPub = rospy.Publisher ('mavros/setpoint_raw/local', PositionTarget, queue_size=1)
        self.rate = rospy.Rate (5.0)

    ## Drone State callback
    def stateCb(self, msg):
        self.state = msg

    def updateSp(self):
        self.sp.position.x = localX
        self.sp.position.y = localY

class Move:
    localposepub=rospy.Publisher('mavros/setpoint_velocity/cmd_vel_unstamped',Twist,queue_size=1)
    information_pub=Twist()
    vel_msg = PositionTarget()
    velocity_pub =rospy.Publisher('mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    def MoveX(self, distance,speed):
        global previous_longitude, previous_latitude, longitude, latitude
        global color_found
        previous_latitude = latitude
        previous_longitude = longitude
        self.vel_msg.header.stamp = rospy.get_rostime()
        self.vel_msg.header.frame_id ="world"
        self.vel_msg.coordinate_frame =8
        self.vel_msg.type_mask = int('011111000111', 2)
        self.vel_msg.velocity.y = speed

        while not rospy.is_shutdown():
            haversine_distance = haversine()
            if (haversine_distance >= distance)|(color_found==True):
                break
            self.velocity_pub.publish(self.vel_msg)
        print("Sa")
        self.vel_msg.velocity.y = 0.
        for i in range(100):
            self.velocity_pub.publish(self.vel_msg)


    def MoveY(self, distance,speed):
        global previous_longitude, previous_latitude, longitude, latitude
        global color_found
        previous_latitude = latitude
        previous_longitude = longitude
        self.vel_msg.header.stamp = rospy.get_rostime()
        self.vel_msg.header.frame_id ="world"
        self.vel_msg.coordinate_frame =8
        self.vel_msg.type_mask = int('011111000111', 2)
        self.vel_msg.velocity.x = speed

        while not rospy.is_shutdown():
            haversine_distance = haversine()
            if (haversine_distance >= distance)|(color_found==True):
                break
            self.velocity_pub.publish(self.vel_msg)
        print("Sa")
        self.vel_msg.velocity.x = 0.
        for i in range(100):
            self.velocity_pub.publish(self.vel_msg)


def moveDown(alt):

    global altitude1
    print ("************ ALCALIYOR ************")
    rate = rospy.Rate (10.0)
    altitudeSp = alt
    msg2.linear.z = -1.5

    while not rospy.is_shutdown ():

        print ("SUANKI YUKSEKLIK=", altitude1)
        zVelocityPub.publish (msg2)
        rate.sleep ()

        if (altitudeSp + 0.15) > altitude1:
            print ("ALCALDIGI DEGER=", altitude1)
            break

    msg2.linear.z = 0

    for i in range (100):
        zVelocityPub.publish (msg2)
    rate.sleep ()


def moveUp():

    global altitude1
    print ("************ YUKSELIYOR ************")
    rate = rospy.Rate (10.0)
    altitudeSp = 8
    msg2.linear.z = 3

    while not rospy.is_shutdown ():

        print ("SUANKI YUKSEKLIK=", altitude1)
        if (altitudeSp - 0.15) < altitude1:
            print ("YUKSELDIGI DEGER=", altitude1)
            break

        zVelocityPub.publish (msg2)
        rate.sleep ()

    msg2.linear.z = 0

    for i in range (100):
        zVelocityPub.publish (msg2)
    rate.sleep ()


def moveCenter():
    global konum, msg1, velocityPub, farkx, farky, redLongitude2, redLatitude2, longitude, latitude
    modes = fcuModes ()
    rate = rospy.Rate (5)
    while 1:

        #uncomment to moveCenter during to winding (0.5 slow, 0.1 fast)
        #v = (0.1 + (0.000833 * konum)) #Vmax =0.6
        v = (0.05 + (0.0009167 * konum)) #Vmax=0.6
        dist=19

        if konum > 20:

            msg1.velocity.z = 0
            msg1.header.stamp = rospy.get_rostime ()
            msg1.header.frame_id = "local_ned"
            msg1.coordinate_frame = 8
            msg1.type_mask = int ('011111000111', 2)

            if farky <= -dist:
                    msg1.velocity.x = v
                    velocityPub.publish (msg1)
                    rate.sleep ()

            elif farky >= dist:
                    msg1.velocity.x = -v
                    velocityPub.publish (msg1)
                    rate.sleep ()

            elif -dist < farky < dist:
                    msg1.velocity.x = 0
                    velocityPub.publish (msg1)
                    rate.sleep ()

            if farkx <= -dist :
                msg1.velocity.y = v
                velocityPub.publish (msg1)
                rate.sleep ()

            elif farkx >= dist:
                msg1.velocity.y = -v
                velocityPub.publish (msg1)
                rate.sleep ()

            elif -dist < farkx < dist:
                msg1.velocity.y = 0
                velocityPub.publish (msg1)
                rate.sleep ()

        elif konum <= 20:
            msg1.velocity.z = 0
            msg1.velocity.y = 0
            msg1.velocity.x = 0
            msg1.yaw = 0  # rad
            msg1.yaw_rate = 0
            velocityPub.publish (msg1)
            rate.sleep ()
            break


def moveWaypoint():
    global first_longitude,first_latitude
    rate = rospy.Rate (20.0)
    global redLongitude, redLatitude
    modes = fcuModes ()
    move = Move()
    globalPositionPublish (first_latitude, first_longitude , 0)
    move.MoveX(10,-1.5)
    move.MoveY(6,1.5)
    move.MoveX(10,1.5)
    move.MoveY(6,1.5)
    move.MoveX(10,-1.5)
    moveCenter()
    moveDown (6)
    modes.setLoiterMode()
    rospy.sleep(10)
    modes.setOffboardMode ()
    moveUp ()
    globalPositionPublish (first_latitude, first_longitude , 0)
    modes.setLandMode ()


# Main function
def main():
    global spPub, spGlobPub, amsl
    # initiate node
    rospy.init_node ('setpoint_node', anonymous=True)

    # flight mode object
    modes = fcuModes ()

    # controller object
    cnt = Controller ()

    # ROS loop rate
    rate = rospy.Rate (5.0)

    # Subscribe to drone state
    rospy.Subscriber ('mavros/state', State, cnt.stateCb)
    rospy.Subscriber ("/mavros/global_position/raw/fix", NavSatFix, globalPositionCallback)
    rospy.Subscriber ('mavros/local_position/pose', PoseStamped, localPositionCallback)
    rospy.Subscriber ('radius', Float64, image_callback)
    rospy.Subscriber ('konum', camera, cam_konum_callback)
    rospy.Subscriber ('mavros/altitude', Altitude, amslcallback)
    rospy.Subscriber ('mavros/altitude', Altitude, altitudeCallback)

    # Setpoint publisher
    spGlobPub = rospy.Publisher ('mavros/setpoint_raw/global', GlobalPositionTarget, queue_size=1)
    spPub = rospy.Publisher ('mavros/setpoint_raw/local', PositionTarget, queue_size=1)

    # Make sure the drone is armed
    while not cnt.state.armed:
        modes.setArm ()
        rate.sleep ()

    modes.setTakeoff ()
    rospy.sleep (10)
    print ("TAKEOFF ALTITUDE=", amsl)

    # activate OFFBOARD mode
    modes.setOffboardMode ()
    print ("************ OFFBOARD MODE ************")
    pose(0, 0, 0)
    moveWaypoint ()

if __name__ == '__main__':
    try:
        main ()
    except rospy.ROSInterruptException:
        pass

