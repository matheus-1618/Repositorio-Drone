#!/usr/bin/env python3

# Quando executado no SSD com o ROS Melodic, 
# a primeira linha deve invocar o programa `python`

from __future__ import absolute_import
from __future__ import division
from __future__ import print_function
from __future__ import unicode_literals

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import Empty

from cv_bridge import CvBridge, CvBridgeError
bridge = CvBridge()

camera_topic = "/bebop2/camera_base/image_raw"
# Descomente a linha abaixo para o bebop real
# camera_topic = "/bebop/image_raw"

magentaLower =(120, 90, 50)
magentaUpper = (180, 255, 255)

amarelo_b= (18,50,50)
amarelo_a = (21,255,255)

verde_b = (70,50,50)
verde_a = (80,255,255)

azul_b = (105, 50, 50)
azul_a  = (110, 255, 255)

linear_speed_factor = 950
angular_speed_factor = -0.0010

MAGENTA = 0
AZUL = 1
AMARELO = 2
VERDE = 3

class StateMachine :

    def __init__(self):
        self.flag = 0
        self.error = 0
        self.max_area = 0
        self.area = 0
        self.state = MAGENTA
        self.lower = magentaLower
        self.up = magentaUpper

        self.pub = rospy.Publisher('/bebop/cmd_vel', Twist, queue_size=10)
        self.camsub = rospy.Subscriber(camera_topic, Image, self.image_callback)

        self.takeoff = rospy.Publisher('/bebop/takeoff', Empty, queue_size=10)
        self.land = rospy.Publisher('/bebop/land', Empty, queue_size=10)
    
    def set_state(self,color):
        self.state = color

    def takeoff_init(self):
        self.takeoff.publish(Empty())

    def land_init(self):
        self.land.publish(Empty())

    def image_callback(self, message):
        
        frame = bridge.imgmsg_to_cv2(message, "bgr8")

        cv2.namedWindow("Frame", cv2.WINDOW_NORMAL)
        cv2.imshow("Frame", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            cv2.destroyAllWindows()
   
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        if self.state == MAGENTA:
            mask = cv2.inRange(hsv, self.lower, self.up)
        
        mask = cv2.erode(mask, None, iterations=2)
        mask = cv2.dilate(mask, None, iterations=2)
        cv2.imshow("Mask", mask)

        contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        objects = np.zeros([frame.shape[0], frame.shape[1], 3], 'uint8')

        # move, draw contours
        max_c= None
        max_c_area= 0
        x=0;
        y=0;
        for c in contours:
            self.area = cv2.contourArea(c)
            if self.area > 30:
                if self.area>max_c_area:
                    max_c = c
                    max_c_area = self.area
                    perimeter = cv2.arcLength(c, True)
                    # now we want to draw the centroid, use image moment
                    # get centers on x and y
                    ((x, y), radius) = cv2.minEnclosingCircle(c)
                    x = int(x)
                    y = int (y)
                    radius = int(radius)
                    cv2.drawContours(frame, [max_c], -1, [0, 0, 255], 3)
                    self.error = x-frame.shape[1]/2
            

        
        self.max_area = max_c_area
        
        cv2.imshow("Contours", frame)

    def color_control(self):
        velocity_message = Twist()
        if self.area <30:
            if self.flag ==0:
                velocity_message.linear.z=0
                velocity_message.angular.z=0.5
                self.pub.publish(velocity_message)
            # if self.flag ==1:
            #     ini = rospy.get_time()
            #     while rospy.get_time()-ini <3:
            #         self.land_init()


        else:
            if (self.max_area>1000):
                velocity_message.linear.x = linear_speed_factor/self.max_area
                Az = self.error * angular_speed_factor ;
                print('max_c_area= ', self.max_area)
                if self.flag ==0:
                    pass
                    #self.flag =1 
                if abs(Az)>0.1:
                    velocity_message.angular.z = Az
                    print('Turning speed ', velocity_message.angular.z)
                else:
                    velocity_message.angular.z =0 
            
                self.pub.publish(velocity_message)

            else:
                velocity_message.linear.x=0
                Az = self.error * angular_speed_factor 
                velocity_message.angular.z=Az
                self.pub.publish(velocity_message)

        return self.state


    def control(self):
        if self.state == MAGENTA:
            self.state = self.color_control()
        elif self.state == AZUL:
            self.state = self.color_control()
        elif self.state == VERDE:
            self.state = self.color_control()
        elif self.state == AMARELO:
            self.state = self.color_control()


if __name__ == '__main__':
    
    rospy.init_node('magenta_tracker',anonymous=True)

    states = StateMachine()
    ini = rospy.get_time()
    while rospy.get_time()-ini < 3:
        states.takeoff_init()

    while not rospy.is_shutdown():
        states.control()
        rospy.sleep(0.1)
