#!/usr/bin/env python

import rospy 
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pi
from std_msgs.msg import Float32
from sensor_msgs.msg import Imu, JointState
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import time

def saturate(data):
    if(data > 1 ):
        return 1
    elif data < -1:
        return -1
    else: 
        return data

def map(data):
    return (data) / 100

class SerialNoSteer:
    def __init__(self):
        # MODES OF SERIAL     
        rospy.init_node("Serial_node" ,anonymous=False)   
        self.is_idle = False
        self.choose_mode = False
        self.is_science = False
        self.is_drive = False
        self.torq_mode = False
        
        self.autonomous = False

        self.rk = False 

        self.time = rospy.Time.now().secs
        
        self.init_pitch = 0  # Initial pitch angle of the rover
        self.last_pitch = 0
        self.curr_pitch = 0
        self.pitch_change = 0
        self.pitch_counter = 0
        self.flag = 0 
        
        # Constants
        self.WHEEL_RADIUS = 0.14
        
        # Data taken from subscribers
        self.axes = None
        self.buttons = None 
       
        self.angular_z = None
        self.lineer_x = None
        
        self.wheel_speeds = Float64MultiArray()

        self.rate = rospy.Rate(5)
        self.init()
        self.run()
        
    def init(self):
        rospy.loginfo("Serial is started")
        self.wheel_speeds.data = [0,0,0,0,0]
        rospy.Subscriber("/joy",Joy,self.joy_cb)
        rospy.Subscriber("/drive_system/twist",Twist, self.twist_cb)
        rospy.Subscriber("/imu1/data", Imu, self.imu_cb)
        self.publisher1 = rospy.Publisher("/drive_system/wheel_speed",Float64MultiArray,queue_size=10) # Drive System RPM data array ?? 
        self.publisher2 = rospy.Publisher("/multiarray_topic2",Float64MultiArray,queue_size=10) # Science System data array ??
        
        
    def switch_mode(self):
        if(self.buttons[8]):
            self.is_idle = True
            self.is_science = False
            self.is_drive = False
            self.choose_mode = False
            self.torq_mode = False
            
        if(self.is_idle== True and self.buttons[7]):
            self.choose_mode = True
            self.is_idle = False
            self.is_science = False
            self.is_drive = False
            self.torq_mode = False
            
        if(self.choose_mode and self.buttons[0]):
            self.is_drive = True
            self.choose_mode = False
            self.is_idle = False
            self.is_science = False
        if(self.choose_mode and self.buttons[3]):
            self.is_science = True
            self.choose_mode = False
            self.is_idle = False
            self.is_drive = False
            self.torq_mode = False
        if(self.is_drive and self.buttons[4] and self.buttons[5] and self.torq_mode):
            self.torq_mode = False
            
        elif(self.is_drive and self.buttons[4] and self.buttons[5] and not self.torq_mode):
            self.torq_mode = True # if you want to use torq mode make it TRUE

    def run(self):
        while not rospy.is_shutdown():
            if(self.is_idle) : 
                rospy.loginfo("IDLE MODE!")
            if(self.is_drive):
                rospy.loginfo("OPERATION MODE")
                if(self.torq_mode):
                    rospy.loginfo("TORQUE MODE ON")
                    self.wheel_speeds.data[4] = True
                self.publisher1.publish(self.wheel_speeds)
                #self.wheel_speeds.data = [0,0,0,0,0]
            if(self.is_science):
                rospy.loginfo("SCIENCE MODE")
                publish1 = Float64MultiArray()
                publish2 = Float64MultiArray()
                publish1.data = self.axes
                publish2.data = self.buttons
                self.publisher1.publish(publish1)
                self.publisher2.publish(publish2)
            if(self.choose_mode):
                rospy.loginfo("CHOOSE A MODE!")
            if(rospy.Time.now().secs - self.time  > 1 ):
                self.wheel_speeds.data = [0,0,0,0,0]
            self.rate.sleep()
            

    def joy_cb(self,data):
        self.axes = data.axes
        self.buttons = data.buttons
        
        self.switch_mode()

        
    def imu_cb(self,data):
        self.last_pitch = self.curr_pitch

        # Read quaternion orientation from IMU and convert to euler angles
        [self.curr_pitch, self.curr_roll, self.curr_yaw] = euler_from_quaternion(
            [
                data.orientation.x,
                data.orientation.y,
                data.orientation.z,
                data.orientation.w,
            ]
        )  # Convert quaternion to euler angles

        #rospy.loginfo("{} {} {}".format((self.curr_roll/3.14)*180, (self.curr_pitch/3.14)*180, (self.curr_yaw/3.14)*180))

        # Calculate initial pitch angle by taking average of first 5 measurements
        if (self.flag == 0) and self.pitch_counter < 5:
            self.pitch_counter += 1
            self.init_pitch += self.curr_pitch

            if self.pitch_counter == 5:
                self.init_pitch /= 5
                self.flag = 1
        else:
            self.pitch_change = (self.curr_pitch - self.init_pitch)/-3.14*180


    def twist_cb(self,data):
        self.lineer_x = data.linear.x
        self.angular_z = data.angular.z
        self.time = rospy.Time.now().secs
        self.wheel_speed_calc()

    def wheel_speed_calc(self):
        if(self.lineer_x != None and self.angular_z != None ):
            
            self.left_wheel = (self.lineer_x - self.angular_z) / 2
            self.right_wheel = (self.lineer_x + self.angular_z) / 2

            self.rpm_left = (self.left_wheel / (2 * pi * self.WHEEL_RADIUS)) * 60
            self.rpm_right = (self.right_wheel / (2 * pi * self.WHEEL_RADIUS)) * 60
            

            self.rpm_left = float(saturate(map(self.rpm_left)))
            self.rpm_right = float(saturate(map(self.rpm_right)))
                            
            self.wheel_speeds.data = [ self.rpm_left, self.rpm_right, self.torq_mode, self.pitch_change]
                
            if(self.autonomous):
                    self.wheel_speeds.data = [self.rpm_left*2,self.rpm_right*2, self.torq_mode, self.pitch_change]


#left_wheel + right_wheel = linear_x
#right_wheel - left_wheel = angular_z



if __name__ == "__main__":
    SerialNoSteer()
        
