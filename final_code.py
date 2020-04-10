# coding: utf8

import math #math
import rospy #main
from clever import srv #idk
from pyzbar import pyzbar #qr
import cv2 as cv #for color recognition
from cv_bridge import CvBridge #qr
from std_srvs.srv import Trigger #arm
from sensor_msgs.msg import Image #qr
from clever.srv import SetLEDEffect #led
from aruco_pose.msg import MarkerArray #aruco_map
from mavros_msgs.srv import CommandBool #arm
import numpy as np #numpy array for color recognition

#to get image from camera
bridge = CvBridge()

#for led control
set_effect = rospy.ServiceProxy('led/set_effect', SetLEDEffect)

rospy.init_node('flight_qr_and_color')

#rospy publishers
image_pub = rospy.Publisher('~qr', Image)
image_pub1 = rospy.Publisher('~color', Image)

#class to store patient information
class patient:
    x = 0.0
    y = 0.0
    temp = -1 # 0 - normal; 1 - question; 2 - high
    sickness = -1 # 0 - healthy; 1 - COVID-19; 2 - non-COVID-19

#for navigate_wait
def get_distance(x1, y1, z1, x2, y2, z2):
    return math.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2 + (z1 - z2) ** 2)

#navigate while drone dont reach coordinate
def navigate_wait(x, y, z, speed, tolerance=0.2):
    navigate(x=x, y=y, z=z, speed=speed, frame_id='aruco_map')
    while not rospy.is_shutdown():
        telem = get_telemetry(frame_id='aruco_map')
        if get_distance(x, y, z, telem.x, telem.y, telem.z) < tolerance:
            break
        rospy.sleep(0.2)

#fly to patient 
def flyToPatient(px, py, spd):
    navigate_wait(px, py, 1.0, spd)

#return to base
def flyToBase(spd):
    navigate_wait(0.0, 0.0, 1.0, spd)

#this function recognize qr code
allPat = []
def image_callback(data):
    temp = patient()
    cv_image = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image
    barcodes = pyzbar.decode(cv_image)
    b_data = ""
    for barcode in barcodes:
        b_data = barcode.data.encode("utf-8")
        b_type = barcode.type
        (x, y, w, h) = barcode.rect
        xc = x + w/2
        yc = y + h/2  
    if b_data == "healthy" and temp.sickness == -1:
        #print("IN healthy if")
        temp.sickness = 0
        allPat.append(temp)
        cv.putText(cv_image, 'Healthy', (50, 50), cv.FONT_HERSHEY_SIMPLEX , 1, (100, 238, 20) , 2, cv.LINE_AA) 
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8')) #bgr8
	return
    if b_data == "COVID - 19" and temp.sickness == -1:
        temp.sickness = 1
	allPat.append(temp)
        cv.putText(cv_image, 'COVID-19', (50, 50), cv.FONT_HERSHEY_SIMPLEX , 1, (48, 48, 245) , 2, cv.LINE_AA) 
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8')) #bgr8
	return
    if b_data == "Non COVID - 19" and temp.sickness == -1:
	temp.sickness = 2
	allPat.append(temp)
        cv.putText(cv_image, 'non-COVID', (50, 50), cv.FONT_HERSHEY_SIMPLEX , 1, (9, 238, 222) , 2, cv.LINE_AA) 
        image_pub.publish(bridge.cv2_to_imgmsg(cv_image, 'bgr8')) #bgr8
	return

#this function check qr codes
def checkPatientVirus(p):
    #print("HERE1")
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback, queue_size=1)
    rospy.sleep(2)
    image_sub.unregister()
    return
colorGetted = []

#this function recognize color using masks
def image_callback1(data):
    rred = 0
    ggreen = 0
    yyellow = 0
    for i in range(10):
        img = bridge.imgmsg_to_cv2(data, 'bgr8')  # OpenCV image

        #get hsv of img
        hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)

        #lower and upper for all of colors
        lower_red = np.array([165, 70, 158])
        upper_red = np.array([255, 209, 255])

        lower_green = np.array([26, 28, 60])
        upper_green = np.array([135, 162, 255])

        lower_yellow = np.array([10, 80, 88])
        upper_yellow = np.array([49, 220, 225])

        #masks for all of colors
        redMask = cv.inRange(hsv, lower_red, upper_red)
        greenMask = cv.inRange(hsv, lower_green, upper_green)
        yellowMask = cv.inRange(hsv, lower_yellow, upper_yellow)

        #img & mask
        redImg = cv.bitwise_and(img, img, mask=redMask)
        greenImg = cv.bitwise_and(img, img, mask=greenMask)
        yellowImg = cv.bitwise_and(img, img, mask=yellowMask)

        #get img center and any pixel without a mask for each color
        imgCenter = (img.shape[0] / 2, img.shape[1] / 2)
        redx, redy = np.where(redMask != 0)
        greenx, greeny = np.where(greenMask != 0)
        yellowx, yellowy = np.where(yellowMask != 0)
        
        #img & all color masks
        maskAll = redMask + yellowMask + greenMask
        img = cv.bitwise_and(img, img, mask=maskAll)

        #check colors
        if imgCenter[0] in redx and imgCenter[1] in redy:
            rred = rred + 1
        elif imgCenter[0] in greenx and imgCenter[1] in greeny:
            ggreen = ggreen + 1
        elif imgCenter[0] in yellowx and imgCenter[1] in yellowy:
            yyellow = yyellow + 1

    if rred > ggreen and rred > yyellow:
        colorGetted.append(1)
        cv.putText(img, 'Red', (50, 50), cv.FONT_HERSHEY_SIMPLEX , 1, (48, 48, 245) , 2, cv.LINE_AA) 
    elif ggreen > rred and ggreen > yyellow:
        colorGetted.append(2)
        cv.putText(img, 'Green', (50, 50), cv.FONT_HERSHEY_SIMPLEX , 1, (100, 238, 20) , 2, cv.LINE_AA) 
    elif yyellow > rred and yyellow > ggreen:
        colorGetted.append(3)
        cv.putText(img, 'Yellow', (50, 50), cv.FONT_HERSHEY_SIMPLEX , 1, (9, 238, 222) , 2, cv.LINE_AA)
    else:
    	colorGetted.append(3)
        cv.putText(img, 'Yellow', (50, 50), cv.FONT_HERSHEY_SIMPLEX , 1, (9, 238, 222) , 2, cv.LINE_AA)
    image_pub1.publish(bridge.cv2_to_imgmsg(img, 'bgr8')) #bgr8
    return 
	
#this function get color
def getColor():
    rospy.sleep(2)
    print("IN GET COLOR") #debug
    image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback1, queue_size=1)
    rospy.sleep(4)
    image_sub.unregister()
    got = colorGetted[-1]
    if got == -1:
        image_sub = rospy.Subscriber('main_camera/image_raw_throttled', Image, image_callback1, queue_size=1)
        rospy.sleep(4)
        image_sub.unregister()
        got = colorGetted[-1]
    print("GOT: ", got) #debug 

    print("LEFT GET COLOR") #debug
    return

arming = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
get_telemetry = rospy.ServiceProxy('get_telemetry', srv.GetTelemetry)
navigate = rospy.ServiceProxy('navigate', srv.Navigate)
navigate_global = rospy.ServiceProxy('navigate_global', srv.NavigateGlobal)
set_position = rospy.ServiceProxy('set_position', srv.SetPosition)
set_velocity = rospy.ServiceProxy('set_velocity', srv.SetVelocity)
set_attitude = rospy.ServiceProxy('set_attitude', srv.SetAttitude)
set_rates = rospy.ServiceProxy('set_rates', srv.SetRates)
land = rospy.ServiceProxy('land', Trigger)	

#set and store coordinates for all of patients
patients = []

p1 = patient()
p2 = patient()
p3 = patient()
p4 = patient()
p5 = patient()
p6 = patient()
p7 = patient()
p8 = patient()
p9 = patient()

p1.x = 0.295 #1
p1.y = 0.295
patients.append(p1)

p2.x = 0.885     #2
p2.y = 0.295
patients.append(p2)

p4.x = 0.885 #3
p4.y = 0.885
patients.append(p4)

p3.x = 0.295 #4
p3.y = 0.885
patients.append(p3)

p5.x = 0.295 #5
p5.y = 1.475
patients.append(p5)

p6.x = 0.885 #6
p6.y = 1.475
patients.append(p6)

p8.x = 0.885 #7
p8.y = 2.065
patients.append(p8)


p7.x = 0.295 #8
p7.y = 2.065
patients.append(p7)

p9.x = 0.59  #9
p9.y = 2.655
patients.append(p9)

#array to store all patients with high or undetected temperature
hotPatients = []

get_telemetry(frame_id='map')

navigate(z=1.0, speed=0.3, frame_id='body', auto_arm=True)

rospy.sleep(1)

get_telemetry(frame_id="aruco_map")

navigate(z=1.0, speed=0.3, frame_id='aruco_map') 
rospy.sleep(10)

#fly from 1 to 0 patients and detect color
#also remember all coordinats of patients with red and yellow markers

#TODO FLIGHT DONE
#TODO RECOGNIZE COLOR DONE
#TODO append all yellow and red to hotPatients.append() DONE
colorGetted.append(-1)

#fly to patients in for
for p in patients:
    flyToPatient(p.x, p.y, 0.3)
    rospy.sleep(3)
    getColor()
    got = colorGetted[-1]
    if got == 1: #red
         print("RED")
         print(">>> HIGH TEMPERATURE +")
	 p.temp = 2
	 hotPatients.append(p)
	 set_effect(effect='flash', r=220, g=0, b=221)
	 rospy.sleep(3)
	 set_effect(r=220, g=20, b=250)
         print(">>> DELIVERED! :)")
    elif got == 2: #green
        print("GREEN")
        print(">>> NORMAL TEMPERATURE -")
	p.temp = 0
        p.sickness = 0
    elif got == 3: #yellow
        print("YELLOW")
        print(">>> NEED A CHECK ?")
	p.temp = 1
	hotPatients.append(p)
	set_effect(effect='flash', r=220, g=0, b=221)
	rospy.sleep(3)
	set_effect(r=220, g=20, b=250)
        print(">>> DELIVERED! :)")


print("fly back to base") #debug
flyToBase(0.3)

#sleep 2 min
print("SLEEEP 120 sec") #debug
land()
rospy.sleep(5)
arming(False)
rospy.sleep(120)
print("fly again") #debug

#after sleep
navigate(z=1.0, speed=0.3, yaw=0.0, frame_id='body', auto_arm=True)
rospy.sleep(1)
navigate(z=1.0, speed=0.3, frame_id='aruco_map')
rospy.sleep(5)
#fly again to ill patients and detect qr codes

#TODO FLIGHT DONE
#TODO RECOGNIZE QR CODES DONE

temp = patient()
temp.sickness = 0

#flying to all hotPatients
for p in hotPatients:
        allPat.append(temp)
	flyToPatient(p.x, p.y, 0.3)
	print(">>> PATIENT: ", p.x, " ", p.y)
	checkPatientVirus(p)
	p = allPat[-1]
	#print(">>> P SIKNESS: ", p.sickness)
	#print(">>>> temp sikness: ", temp.sickness)
	if p.sickness == 0:
	    print(">>> HEALTHY")
	elif p.sickness == 1:
	    print(">>> COVID - 19")
	    set_effect(r=255, g=0, b=0)  # fill strip with red color
	    rospy.sleep(5)
	    set_effect(r=220, g=20, b=250)
	elif p.sickness == 2:
	    print(">>> Non - COVID-19")

print("fly back to base") #debug
flyToBase(0.3)

#final landing
print("going to land") #debug

land()
rospy.sleep(5)
arming(False)

print("OUTPUT EVERYTYHING ABOUT PATIENTS:")
for p in patients:
    print("Patient x: {} Patient y: {} Patient temp: {}").format(p.x, p.y, p.temp)

print("END OF PROGRAMM! HURRAY!! BlackCat_NTI_2020")
