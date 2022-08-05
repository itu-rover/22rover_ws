#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy 
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class LedControl : 
    def __init__(self):
        rospy.init_node("led")
        
        self.pub = rospy.Publisher("/led_topic",String,queue_size=10)
         
        self.flag_nav = False
        self.time_nav = rospy.Time.now().secs
        
        self.flag_joy = False
        
        rospy.set_param("/mydelay", 1.5)
        self.time_joy = rospy.Time.now().secs
        if rospy.has_param("/mydelay"):
            self.param = rospy.get_param("/mydelay")
        else:
            self.param = 0.5
        
        rospy.Subscriber("/joystick/twist",Twist,self.joy_cb)
        rospy.Subscriber("/nav_vel",Twist,self.nav_cb)
        
        
        self.rate = rospy.Rate(5)
        self.run()       
       
       
       
    def joy_cb(self,data):
        self.flag_joy = True
        self.time_joy = rospy.Time.now().secs
        
        
        
        
    def nav_cb(self,data):
        self.flag_nav = True
        self.time_nav = rospy.Time.now().secs



    def run(self):
        
        while not rospy.is_shutdown():
            
            if(self.flag_joy):
                self.pub.publish("G"),
                print("Green")

            elif(self.flag_nav):
                self.pub.publish("R")
                print("Red")
                
            else :
                self.pub.publish("Y")
                print("Yellow")
                
                
            if(int(rospy.Time.now().secs) - int(self.time_joy) > self.param):
               self.flag_joy = False 
               
            if(int(rospy.Time.now().secs) - int(self.time_nav) > self.param):
                self.flag_nav = False
                
            self.rate.sleep()
            
            
            


if __name__ == "__main__":
    LedControl()