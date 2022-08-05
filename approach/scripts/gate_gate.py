#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray
from geometry_msgs.msg import Transform,PoseStamped
from nav_msgs.msg import Odometry 
import math as m
import numpy as np
from tf.transformations import euler_from_quaternion,quaternion_from_euler
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Bool


class Gate:
    def __init__(self) :
        #rospy.init_node("Gate")
        self.x,self.y,self.z = None,None,None
        rospy.Subscriber("/lio_sam/mapping/odometry",Odometry,self.odom)
        rospy.Subscriber("/fiducial_transforms",FiducialTransformArray,self.marker)
        rospy.Subscriber("/locomove_base/status",GoalStatusArray,self.move_base_status)
        self.pub_e_stop = rospy.Publisher("/e_stop",Bool,queue_size=10)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=10)

        self.goal_number = 2
        self.initalized_look = False # need to find points just once if points are found then it will be true
        self.initalized_finish = False
        self.flag_draw_circle = False
        self.done = False
        self.stage = 0
        self.yaw = 0
        self.circle_pts = []
        self.markers = {}
        self.count_marker = {}
        self.main_pts = [] # look points
        self.trial = 0
        self.starting_time = rospy.Time.now().to_sec()
        self.Rate = rospy.Rate(1)
        self.status = 1
        self.run()

    def odom(self,data):
        self.x = data.pose.pose.position.x
        self.y = data.pose.pose.position.y
        self.z = data.pose.pose.position.z
        self.x_or = data.pose.pose.orientation.x
        self.y_or = data.pose.pose.orientation.y
        self.z_or = data.pose.pose.orientation.z
        self.w_or = data.pose.pose.orientation.w
        self.roll,self.pitch,self.yaw = euler_from_quaternion((self.x_or,self.y_or,self.z_or,self.w_or))

    def run(self):
        while not rospy.is_shutdown():
            if self.done == False:
                if rospy.Time.now().to_sec() - self.starting_time >2 :
                    #if (rospy.Time.now().to_sec() - self.starting_time > 20) and (len(self.markers) == 0 ):
                        #return False 
                    if (self.initalized_look == False and len(self.markers) == 1):
                        print("saw only one tag")
                        self.marker_x,self.marker_y = list(self.markers.values())[0].translation.x,list(self.markers.values())[0].translation.y
                        self.circle_points((self.marker_x,self.marker_y),5,m.pi/3)
                        self.initalized_look = True
                        self.flag_draw_circle = True
                    if(self.initalized_look and len(self.markers) == 1 ):
                        self.draw_circle()
                        print("looking for second")
                    if(len(self.markers) == 2 and not self.initalized_finish):
                        print("found both")
                        rospy.sleep(2)
                        self.gate_points(3)
                        self.initalized_finish = True
                        print("start_finish")
                    if(self.initalized_finish and len(self.markers) == 2):
                        self.finish_job()
                    self.Rate.sleep()
            else:
                return True


    def gate_points(self,r):
        middle_x =  (list(self.markers.values())[0].translation.x + list(self.markers.values())[1].translation.x) / 2
        middle_y =  (list(self.markers.values())[0].translation.y + list(self.markers.values())[1].translation.y) / 2
        self.middle_p = (middle_x,middle_y)
        angle = m.atan2((list(self.markers.values())[0].translation.y - list(self.markers.values())[1].translation.y),(list(self.markers.values())[0].translation.x - list(self.markers.values())[1].translation.x))
        angle += m.pi/2
        self.main_pts.append((m.cos(angle) * r + middle_x ,m.sin(angle) * r + middle_y))
        self.main_pts.append((-m.cos(angle) * r + middle_x ,-m.sin(angle) * r + middle_y))
        dist1 = self.find_distance((self.x,self.y),self.main_pts[0])
        dist2 = self.find_distance((self.x,self.y),self.main_pts[1])
        if(dist1 < dist2):
            self.first_x,self.first_y = self.main_pts[0][0],self.main_pts[0][1]
            self.next_x,self.next_y = self.main_pts[1][0],self.main_pts[1][1]
        else:
            self.first_x,self.first_y = self.main_pts[1][0],self.main_pts[1][1]
            self.next_x,self.next_y = self.main_pts[0][0],self.main_pts[0][1]
       
            


    def find_distance(self,pt1,pt2):
        return np.sqrt((pt1[0]-pt2[0])**2 + (pt1[1]-pt2[1])**2)

    def finish_job(self):
        if self.stage == 0:
            self.stage+=1
            self.send_goal(self.first_x,self.first_y,m.atan2(self.middle_p[1]-self.first_y,self.middle_p[0]-self.first_x))
        # elif self.stage == 0:
        #     self.stage+=1
        #     self.send_goal(self.x,self.y,m.atan2(self.middle_p[1]-self.y,self.middle_p[0]-self.x))
        
        self.gate_points(3)
        
        if(self.stage == 1):
            self.send_goal((self.middle_p[0]+self.x) / 2 , (self.middle_p[1]+self.y) / 2 ,self.yaw)
            self.stage+=1
            print("1")
        if(self.status == 3 and self.stage == 2):
            self.stage+=1
            print("2")
            self.send_goal(self.middle_p[0],self.middle_p[1],self.yaw)
        if(self.status == 3 and self.stage == 3):            
            self.send_goal(self.next_x,self.next_y,self.yaw)
            print("3")
            self.stage +=1

    def stop(self):
        rospy.loginfo("I stopped")
        for i in range(10):
            self.pub_e_stop.publish(True)
        
    def go_on(self):
        rospy.loginfo("I go on")
        for i in range(10):
            self.pub_e_stop.publish(False)
    


    def find_first_p(self,p1,p2,r):
        x1,y1 = p1 # vehicle
        x2,y2 = p2 # marker
        angle = m.atan2(y2-y1,x2-x1)
        relative_y = m.sin(angle) * r
        relative_x = m.cos(angle) * r
        self.send_goal(self.x,self.y,angle -m.pi/2)
        return(relative_x ,relative_y)
        

    
    def marker(self,data):
        for i in data.transforms:
            self.correct_transform = Transform()
            transform = i.transform
            self.correct_transform.translation.x = transform.translation.z
            self.correct_transform.translation.y = - transform.translation.x
            self.correct_transform.rotation = transform.rotation
            if i.fiducial_id not in self.count_marker.keys():
                self.count_marker[i.fiducial_id] =0
                self.stop()
                print("I might see a tag just checking!")
                rospy.sleep(5)
                self.go_on()
            for key,value in self.count_marker.items():
                if key == i.fiducial_id:
                    self.count_marker[i.fiducial_id] +=1
                if value > 5:
                    self.correct_transform.translation.x += 0.3 # since camera is 50 cm in front of base link
                    #print("marker : " + str(pose.translation.x)  + "  " + str(pose.translation.y))    
                    dist_from_vehicle = self.find_distance((0,0),(self.correct_transform.translation.x,self.correct_transform.translation.y))  
                    #print(dist_from_vehicle)
                    beta = m.pi -  m.atan2(self.correct_transform.translation.x,self.correct_transform.translation.y)
                    #print(beta)
                    #print(self.yaw)
                    alpha = self.yaw -  (m.pi/2  - beta)
                    #print(alpha)
                    x_rel = (dist_from_vehicle ) * m.cos(alpha)
                    y_rel = (dist_from_vehicle ) * m.sin(alpha)
                    #print("marker : " + str( x_rel)  + "  " + str(y_rel))
                    if(self.x != None):
                        self.correct_transform.translation.x = x_rel + self.x
                        self.correct_transform.translation.y = self.y +  y_rel
                        #print("marker last : " + str( pose.translation.x)  + "  " + str(pose.translation.y))
                        self.markers[i.fiducial_id] = self.correct_transform
                    break
        #print("I am sure there are " + str(len(self.markers)) + " markers" )
    
    def circle_points(self,center_p,r,angle):
        self.circle_angle = angle
        rotation_matrix = np.array(([m.cos(angle),-m.sin(angle)],[m.sin(angle),m.cos(angle)]))
        first_p = self.find_first_p((self.x,self.y),center_p,r)
        last_p = np.array((first_p))
        for i in range(int(2 * m.pi / angle) + 2):
            last_p = np.matmul(rotation_matrix,last_p)
            self.circle_pts.append((last_p[0] + center_p[0],last_p[1] + center_p[1]))
        

    def move_base_status(self,data):
        self.stat_list = data.status_list
        for i in self.stat_list :
            self.status = i.status
            self.last_goal_id = i.goal_id
        
    
    def draw_circle(self):
        if (self.status == 3 and  (self.flag_draw_circle)) :
            rospy.loginfo("Get a new Goal")
            self.goal_number += 1
            x,y = self.circle_pts[self.goal_number]
            angle = m.atan2(self.marker_y - y,self.marker_x-x) 
            self.send_goal(x,y,angle)

    def create_geo_pose_stamped(self,x, y, yaw, frame_id):
        pose = PoseStamped()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z,pose.pose.orientation.w = quaternion_from_euler(0,0,yaw)
        pose.header.frame_id = frame_id
        self.last_stamp =  rospy.Time.now()
        pose.header.stamp =self.last_stamp
        return pose

    def send_goal(self,x, y, yaw, frame_id='odom'):
        pose = self.create_geo_pose_stamped(x,y,yaw,frame_id)
        print(pose)
        self.goal_pub.publish(pose)
        self.status = 1

if __name__ == "__main__":
    Gate()
