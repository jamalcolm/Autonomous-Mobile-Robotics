# -*- coding: utf-8 -*-
"""
Created on Tue Mar 12 18:33:41 2019
@author: Jason Malcolm
"""

import numpy

import cv2
import cv_bridge
import rospy

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseWithCovarianceStamped, Quaternion
from tf.transformations import quaternion_from_euler
from math import radians

#Set of waypoints on the map
waypoints = [
[-4.48, -4.79, -1.73],
[2.96, 4.07, -0.7],
[0.0, 0.0, -0.42],
[-4.29, -0.28, -0.04],
[3.99, -5.36, 2.15], 
[-3.77, 2.97, -0.06],
[-3.74, 4.58, -0.08],
[-0.96, 3.01, 1.56],
[0.28, 4.59, -0.04],
[1.08, 1.91, -0.04],
[2.63, 1.82, -1.57],
[3.99, 4.58, -3.00],
]

#Cylinder data
class Cylinder:
    def __init__(self, hsv_lower=[0.0,0.0,0.0], hsv_upper=[0.0,0.0,0.0], name="default"):
        self.hsv_lower = numpy.array(hsv_lower)
        self.hsv_upper = numpy.array(hsv_upper)
        self.name = name

#Navigator class
#Handles navigation of the map and stores navigation state
class Navigator:
    def __init__(self):
        
        #Navigation state
        self.nav_state = "random"
        self.err = 0.0
        self.map_w = 5.65
        self.map_h = 6.2   
        self.xpos = 0.0
        self.ypos = 0.0
        self.yaw = 0.0
        self.setGoal = False
        self.wayy=0
        
        #Setup navigation
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.get_pose)
        rospy.Subscriber("/odom", Odometry, self.get_pose)
        self.nav_as = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        self.nav_as.wait_for_server()
        rospy.loginfo("Connected!")
        self.secs = 0
        self.thresh = 5
        
        self.update_time()
        self.set_random_goal()
        
    #Update inner clock so can tell if navigation path is taking too long
    def update_time(self):
        time = rospy.get_rostime()
        self.secs = time.secs
        
    #Get current time
    def get_time(self):
        time = rospy.get_rostime()
        secs = time.secs
        return secs
        
    #Get current pose of the robot and update navigation
    def get_pose(self,msg):
        #Get pose
        self.xpos = float(msg.pose.pose.position.x)
        self.ypos = float(msg.pose.pose.position.y)
        self.yaw = float(msg.pose.pose.orientation.w)

        #'random' moves to set waypoints on the map
        if self.nav_state == "random":
            if self.setGoal == True:
                #Set new goal if aborted/failed
                nav_state = self.nav_as.get_state()
                if nav_state == 4 or nav_state == 5:
                    self.setGoal = False
                    print("Goal aborted/failed!")
                    self.set_random_goal()
                #Set new goal if succeeded
                elif nav_state == 3:
                    print("Goal succeeded!")
                    self.setGoal = False
                    self.set_random_goal()
                #Set new goal if took too long
                elif self.get_time() > self.secs + self.thresh:
                    print("Took too long!")
                    self.setGoal = False
                    self.set_random_goal()
                    self.thresh = 30
        #Return to random from a different state                
        elif self.nav_state == "random_init":
            self.set_random_goal()
            self.update_time()
            self.nav_state = "random"
        #Cancel goal if not in random state
        else:
            self.cancel_goal()
        
    #Create MoveBasegoal with x,y,yaw
    def create_nav_goal(self, x, y, yaw):
        mb_goal = MoveBaseGoal()
        mb_goal.target_pose.header.frame_id = '/map'
        mb_goal.target_pose.pose.position.x = x
        mb_goal.target_pose.pose.position.y = y
        mb_goal.target_pose.pose.position.z = 0.0
        
        angle = radians(yaw) # convert angle to radians
        quat = quaternion_from_euler(0.0, 0.0, angle) # roll, pitch, yaw
        mb_goal.target_pose.pose.orientation = Quaternion(*quat.tolist())
        self.update_time()        
        
        return mb_goal
        
    def set_random_goal(self): 
        #Ordered arrray of waypoints
        if self.setGoal == False:        
            way = waypoints[self.wayy%len(waypoints)]
            self.wayy+=1
            x = way[0]
            y = way[1]
            yaw = way[2]
            
            nav_goal = self.create_nav_goal(x, y, yaw)
            rospy.loginfo("Sending goal to: x="+str(x)+" y="+str(y)+" yaw="+str(yaw)+"...")
            self.nav_as.send_goal(nav_goal)
            self.setGoal = True
        
    #Set goal to specific area    
    def set_coord_goal(self, x, y, yaw):
        if self.setGoal == False:
            nav_goal = self.create_nav_goal(x, y, yaw)
            rospy.loginfo("Sending goal to: x="+str(x)+" y="+str(y)+" yaw="+str(yaw)+"...")
            self.nav_as.send_goal(nav_goal)
            self.setGoal = True
        
    #Cancel goal if there is a current goal
    def cancel_goal(self):
        if self.setGoal:
            self.nav_as.cancel_goal();
            rospy.loginfo("Cancelled goal")
            self.setGoal = False
            
            
#Follower
#Uses rgb camera and depth perception to locate and move towards cylinders
#Also stores which cylinders have been found and their data
class Follower:
    def __init__(self):
        #Initialise navigator
        self.navigator = Navigator()
        
        #Subscribe
        self.twist = Twist()
        self.bridge = cv_bridge.CvBridge()
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)
        self.depth_sub = rospy.Subscriber('/scan', LaserScan, self.depth_callback)
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        self.lookingAt = False        
        
        #Initialise cylinders
        self.cylinders = []
        self.cylinders.append(Cylinder([0,100,100], [1,255,255], "red"))
        self.cylinders.append(Cylinder([30,100,100], [31,255,255], "yellow"))
        self.cylinders.append(Cylinder([60,100,100], [61,255,255], "green"))
        self.cylinders.append(Cylinder([120,100,100], [121,255,255], "blue"))
        
        #Misc.
        self.found = [False, False, False, False]
        self.lookFor = -1
        
        self.secs = 0
        self.secs2 = 0
        self.secs3 = 0
        self.thresh = 5
        self.retry_waiting = False
        
        self.update_time()
        
    #Use camera to locate cylinders    
    def image_callback(self,msg):
        cv2.namedWindow("window", 1)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        showMask = False
        for n in range(0, len(self.cylinders)):
            #Do not look if already found
            if self.found[n]:
                continue
            
            #Do not look if looking for a different cylinder
            if self.lookFor != -1 and self.lookFor != n:
                continue
            
            #Look for cylinder by finding colours within specific range
            hsv_lower = self.cylinders[n].hsv_lower
            hsv_upper = self.cylinders[n].hsv_upper
            mask = cv2.inRange(hsv, hsv_lower, hsv_upper)
            showMask = True
            h, w, d = image.shape
            
            search_top = 0
            search_bot = h
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0
            
            kernel = numpy.ones((50, 20), numpy.uint8)
            mask = cv2.erode(mask, kernel)
            M = cv2.moments(mask)
            
            #If found, cancel current goal and move towards cylinder
            if M['m00'] > 0:
                cx = int(M['m10']/M['m00'])
                self.looking_for(n)
                self.navigator.err = cx - w/2
                if self.navigator.nav_state == "random" and not self.retry_waiting:
                    self.navigator.nav_state = "towards_cylinder_init"
                    self.navigator.cancel_goal();
                    self.update_time()
                    
            #If not found, look for a different cylinder
            else:
                self.lookFor = -1
        
        #Show mask (if looking for cylinder)        
        if showMask:
            cv2.imshow("mask", mask)
            cv2.waitKey(3)
            
        #If it gets stuck, give time to re-orient itself/
        if self.retry_waiting and self.get_time() > self.secs2 + self.thresh:
            self.retry_waiting = False
            print("Reoriented. Will continue to look for cylinders.")
                
    #Zone in and move towards cylinder
    def depth_callback(self,msg):
        scan_mid = len(msg.ranges) / 2
        depth = msg.ranges[scan_mid]
        
        #Zone in
        if self.navigator.nav_state == "towards_cylinder_init":
            self.twist.angular.z = -float(self.navigator.err) / 100.0
            
            if abs(self.navigator.err) < 2.0:
                self.update_time()
                self.navigator.nav_state = "towards_cylinder"
                self.secs3 = self.get_time()
                
            #If takes too long, it may be obstructed or too far away. Continue last navigation path to try and get closer.
            elif self.get_time() > self.secs + self.thresh:
                self.navigator.nav_state = "random_init"
                self.lookFor = -1
                print("Cannot centre in due to obstruction.")
                
                if not self.retry_waiting:
                    self.secs2 = self.get_time()
                    self.get_time();
                    self.retry_waiting = True
                    self.lookFor = -1
                    self.navigator.wayy-=1
                
            else:
                self.twist.linear.x = 0.0
            self.cmd_vel_pub.publish(self.twist)
        
        #Use twist to move towards the cylinder.
        elif self.navigator.nav_state == "towards_cylinder":
            self.twist.angular.z = -float(self.navigator.err) / 100.0
            self.twist.linear.x = 0.5
            self.cmd_vel_pub.publish(self.twist)

            #Report found
            if depth < 1.0 and not numpy.isnan(depth):
                self.found_cylinder()
                self.lookFor = -1
                self.twist.linear.x = 0.0
                self.navigator.nav_state = "random_init"
                
            #Again if it takes too long, attempt to re-orient itself
            elif self.get_time() > self.secs3 + 30:
                self.secs3 = self.get_time()
                self.secs2 = self.get_time()
                self.update_time();
                self.navigator.nav_state = "random_init"
                self.lookFor = -1
                self.retry_waiting = True
                self.navigator.wayy-=1
                print("Cannot centre in due to obstruction.")
                
                    
        else:
            self.twist.angular.z = 0.0
            
    #Declare looknig for a cylinder    
    def looking_for(self,n):
        if self.found[n] == False:
            if self.lookFor != n:
                print("Looking for "+self.cylinders[n].name+" cylinder")
            self.lookFor = n
            
    #Declare cylinder found        
    def found_cylinder(self):
        n = self.lookFor
        if self.found[n] == False:
            self.found[n] = True
            print("Found "+self.cylinders[n].name+" cylinder!\n"+str(self.found))
            self.lookFor = -1
            
            if self.found == [True,True,True,True]:
                print("All cylinders found!!!")
    
    #update time        
    def update_time(self):
        time = rospy.get_rostime()
        self.secs = time.secs
    
    #Get time    
    def get_time(self):
        time = rospy.get_rostime()
        secs = time.secs
        return secs
        
#Main
if __name__ == "__main__":
    cv2.startWindowThread()
    rospy.init_node('follower')
    myState = Follower()
    rospy.spin()
    cv2.destroyAllWindows()
