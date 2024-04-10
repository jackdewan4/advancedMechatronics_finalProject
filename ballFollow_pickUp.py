#!/usr/bin/env python


from __future__ import print_function
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
# imports for manipulator control
import moveit_commander
import moveit_msgs.msg
# imports for initializing navigation position
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
# imports for navigation control
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#import subprocess
import subprocess


class TakePhoto:
    def __init__(self):
        
        
        # self.process = subprocess.Popen(["roslaunch", "turtlebot3_manipulation_navigation", "navigation.launch map_file:=$HOME/mapFinalProj.yaml"])
        self.pub=rospy.Publisher('/cmd_vel',Twist,queue_size=10)
        self.rate=rospy.Rate(1)
        self.rot=Twist()
        
        self.ball_is_taken=False

        self.cx=0
        self.cy=0

        self.bridge = CvBridge()
        self.image_received = False

        # Connect image topic
        img_topic = "/camera/rgb/image_raw"
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)


        # initialize global state flags
        self.goalAcheived = 0
        self.goal2Acheived = 0
        self.goal3Acheived = 0
        self.intermediateGoal2Acheived = 0
        self.oldWidth = 0

        self.area = 0
        self.trajectory1Complete = 0
        self.trajectory2Complete = 0

        # counter for how far to travel

        self.count = 0
        # manipulator variables


        self.names1 = 'position1'
        self.values1 = [-0.155,0.091,0.212,0.280]
        self.names2 = 'position2'
        self.values2 = [-0.027,1.009,-0.725,-0.0221]

        self.names3 = 'position3'
        self.values3 = [-0.031,0.590,-0.227,-0.650]

        self.names4 = 'position4'
        self.values4 = [-0.058,-0.536,0.985,-1.064]

        self.names5 = 'position5'
        self.values5 = [-0.038,0.147,0.474,-0.012]



        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.arm_group = moveit_commander.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.MoveGroupCommander("gripper")
        self.display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        #Had probelms with planner failing, Using this planner now. I believe default is OMPL
        self.arm_group.set_planner_id("RRTConnectkConfigDefault")
        #Increased available planning time from 5 to 10 seconds
        self.arm_group.set_planning_time(20)

        self.arm_group.remember_joint_values(self.names1, self.values1)
        self.arm_group.remember_joint_values(self.names2, self.values2)
        self.arm_group.remember_joint_values(self.names3, self.values3)
        self.arm_group.remember_joint_values(self.names4, self.values4)
        self.arm_group.remember_joint_values(self.names5, self.values5)

        self.gripper_group_variable_values = self.gripper_group.get_current_joint_values()
        
        # initializing navigation set up 
        self.initPosePublisher = rospy.Publisher('/odom', PoseWithCovarianceStamped, queue_size = 1)
        # Construct message
        self.init_msg = PoseWithCovarianceStamped()
        self.init_msg.header.frame_id = "map"

        self.init_msg.pose.pose.position.x = 0
        self.init_msg.pose.pose.position.y = 0
        self.init_msg.pose.pose.orientation.x = 0
        self.init_msg.pose.pose.orientation.y = 0
        self.init_msg.pose.pose.orientation.z = 0
        self.init_msg.pose.pose.orientation.w = 1


        # navigation goal pose set up 


        # Allow up to one second to connection
        rospy.sleep(1)

        # # Publish message to initialize navigation
        rospy.loginfo("setting initial pose")
        self.initPosePublisher.publish(self.init_msg)
        rospy.loginfo("initial pose is set")

        

    def callback(self, data):

        # Convert image to OpenCV format
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image
        
        if(not self.goalAcheived):

            if(self.area < 50000):

                self.find_ball(cv_image)
                self.move_to_object()
            else:
                
                
                self.goalAcheived = 1
                print('goal acheived')
                self.stop()
                
        elif(self.goalAcheived and not self.trajectory1Complete):
            
            if(self.count < 14):
                    self.count += 1
                    self.rot.linear.x=0.2
                    self.pub.publish(self.rot)
            else:

                self.rot.linear.x=0
                self.pub.publish(self.rot)
            
                print('goal sustained and not moving again')
                self.stop()

                # when the robot stops, the sim has some error an the mobile base continues to rotate slightly as it settles,
                # therefore a delay must be added before we begin to execute the trajectory in order to make sure the joint angles can be as precise as possible
                # begin trajectory 1 to pick up first flower
                self.move_home()
                self.open_gripper()
                self.move_position1()
                self.move_position2()
                self.move_position3()
                self.move_position4()

                self.trajectory1Complete = 1
        elif(self.trajectory1Complete and not self.goal2Acheived):

           # add a timer, to rotate for a specified amount of time before trying to find ball
            
            
            
            # again, then once timer is complete search for ball again, initial attempt of trying to measure size of ball and
            # searching based on being close to current ball does not work.

            #  another idea is to navigate to drop off position in next state in order to avoid the ball finding algo, then we can navigate close
            # to the next ball position then find next ball using the algo, continuing with the state machine will work well
            
            # implementing this navigation implementation now

            rospy.loginfo("moving to point 1 \n")
            self.moveToPose(2.7289962126235006, 2.56646599300953, 0, 0, 0, 0.7072993472328462  , 0.7069141626845439)
            rospy.loginfo("successfully moved to point 1 \n")

            rospy.loginfo(navclient.get_result())
            
            self.goal2Acheived = 1
            print('goal 2 acheived')
        elif(self.goal2Acheived and not self.goal3Acheived):

            navclient.cancel_goal()
            navclient.cancel_all_goals()
            
            self.goal3Acheived = 1

                    
        
        elif(self.goal3Acheived and not self.trajectory2Complete):

            print('goal 3 sustained and not moving again')
            self.stop()
            # # begin trajectory 2 to drop off first flower
            
            self.move_position5()
            self.move_home()
            self.trajectory2Complete = 1
            print('trajectory 2 complete')

        elif(self.trajectory2Complete):
            print('ready to to navigate again')
            self.stop()
            
            
    def show_image(self,img):
        cv2.imshow("Image Window", img)
        cv2.waitKey(3)
        
    def find_goal(self,img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv_frame = cv2.resize(hsv_frame,(640,300))
        img = cv2.resize(img,(640,300))

        
        
        low_H=25
        low_S=100
        low_V=100
        high_H=32
        high_S=255
        high_V=255


        mask_frame=cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        cv2.imshow("mask",mask_frame)
        contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #_, contours, _= cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        X,Y,W,H=0,0,0,0


        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            
            if(area > 30):
                
                x, y, w, h = cv2.boundingRect(contour)
                if(w*h>W*H):
                    X, Y, W, H= x, y, w, h

            print('area = ' + str(area))
            self.area = area
        img = cv2.rectangle(img, (X, Y),(X + W, Y + H),(0, 0, 255), 2)

        self.cx = X+(W/2)
        self.cy = Y+(W/2)
        
        if(self.oldWidth >= 1 and self.cx == 0):
            self.goalAcheived = 1

        self.oldWidth = self.cx

        
        print("to goal")
        print(self.cx)
        
        cv2.imshow("window", img)
        cv2.waitKey(3)
        
        
    def find_ball(self,img):
        hsv_frame = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        hsv_frame = cv2.resize(hsv_frame,(640,300))
        img = cv2.resize(img,(640,300))


        low_H=0
        low_S=100
        low_V=100
        high_H=18
        high_S=255
        high_V=255
        
        
        #low_H=25
        #low_S=100
        #low_V=100
        #high_H=32
        #high_S=255
        #high_V=255


        mask_frame=cv2.inRange(hsv_frame, (low_H, low_S, low_V), (high_H, high_S, high_V))
        cv2.imshow("mask",mask_frame)
        contours, hierarchy = cv2.findContours(mask_frame,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        #_, contours, _= cv2.findContours(mask_frame, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        X,Y,W,H=0,0,0,0


        for pic, contour in enumerate(contours):
            area = cv2.contourArea(contour)
            print('initial area captured = ' + str(area))
            if(area > 30):
                
                x, y, w, h = cv2.boundingRect(contour)
                if(w*h>W*H):
                    X, Y, W, H= x, y, w, h
            print('area = ' + str(area))
            self.area = area
        img = cv2.rectangle(img, (X, Y),(X + W, Y + H),(0, 0, 255), 2)

        self.cx = X+(W/2)
        self.cy = Y+(W/2)
        
        if(W>310 and self.cy>240):
        	self.ball_is_taken=True
        	print("Taken")

        print(f"W={W}")
        print(self.cx)
        
        cv2.imshow("window", img)
        cv2.waitKey(3)

    def move_to_object(self):
        
        if(self.cx==0 or self.area > 50000):
            print("searching")
            self.rot.angular.z=0.1
            self.rot.linear.x=0

        # if(self.goalAcheived == 1):
        #     self.stop()

        else:
            
            # if( not self.goalAcheived):

            obj_x=self.cx-320

            if(obj_x<=40 and obj_x>=-40):
                text="straight"
                self.rot.angular.z=0
                self.rot.linear.x=0.1
            elif(obj_x>60):
                text="Left"
                self.rot.angular.z=-0.1
                self.rot.linear.x=0
            elif(obj_x<-60):
                text="Right"
                self.rot.angular.z=0.1
                self.rot.linear.x=0
            
            # if(self.cx > 375):
            #     self.goalAcheived = 1
            
            # if(self.goalAcheived):
            #     print('goal acheived')
                # self.rot.angular.z=0
                # self.rot.linear.x=0

        # if (not self.goalAcheived):
        #     self.stop()


        self.pub.publish(self.rot)
        # print(text)

    def stop(self):
        self.rot.angular.z=0
        self.rot.linear.x=0
        self.pub.publish(self.rot)
        # print(text)

    

    # manipulator control functions

    def open_gripper(self):
        print ("Opening Gripper...")
        self.gripper_group_variable_values[0] = 00.009
        self.gripper_group.set_joint_value_target(self.gripper_group_variable_values)
        plan2 = self.gripper_group.go()
        self.gripper_group.stop()
        self.gripper_group.clear_pose_targets()
        rospy.sleep(1)

    def close_gripper(self):
        print ("Closing Gripper...")
        self.gripper_group_variable_values[0] = -00.0006
        self.gripper_group.set_joint_value_target(self.gripper_group_variable_values)
        plan2 = self.gripper_group.go()
        self.gripper_group.stop()
        self.gripper_group.clear_pose_targets()
        rospy.sleep(1)

    def move_home(self):
        self.arm_group.set_named_target("home")
        print ("Executing Move: Home")
        plan_success, plan1, planning_time, error_code = self.arm_group.plan()
        self.arm_group.execute(plan1, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        variable = self.arm_group.get_current_pose()
        print (variable.pose)
        rospy.sleep(1)

    # def move_zero():
    # 	arm_group.set_named_target("zero")
    # 	print ("Executing Move: Zero")
    # 	plan_success, plan1, planning_time, error_code = arm_group.plan()
    # 	arm_group.execute(plan1, wait=True)
    # 	arm_group.stop()
    # 	arm_group.clear_pose_targets()
    # 	variable = arm_group.get_current_pose()
    # 	print (variable.pose)
    # 	rospy.sleep(1)

    def move_position1(self):
        self.arm_group.set_named_target("position1")
        print ("Executing Move: Position1")
        plan_success, plan1, planning_time, error_code = self.arm_group.plan()
        self.arm_group.execute(plan1, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        variable = self.arm_group.get_current_pose()
        print (variable.pose)
        rospy.sleep(1)

    def move_position2(self):
        self.arm_group.set_named_target("position2")
        print ("Executing Move: Position2")
        plan_success, plan2, planning_time, error_code = self.arm_group.plan()
        self.arm_group.execute(plan2, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        variable = self.arm_group.get_current_pose()
        print (variable.pose)
        rospy.sleep(1)

    def move_position3(self):
        self.arm_group.set_named_target("position3")
        print ("Executing Move: Position3")
        plan_success, plan2, planning_time, error_code = self.arm_group.plan()
        self.arm_group.execute(plan2, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        variable = self.arm_group.get_current_pose()
        print (variable.pose)
        rospy.sleep(1)

    def move_position4(self):
        self.arm_group.set_named_target("position4")
        print ("Executing Move: Position4")
        plan_success, plan2, planning_time, error_code = self.arm_group.plan()
        self.arm_group.execute(plan2, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        variable = self.arm_group.get_current_pose()
        print (variable.pose)
        rospy.sleep(1)

    def move_position5(self):
        self.arm_group.set_named_target("position5")
        print ("Executing Move: Position5")
        plan_success, plan2, planning_time, error_code = self.arm_group.plan()
        self.arm_group.execute(plan2, wait=True)
        self.arm_group.stop()
        self.arm_group.clear_pose_targets()
        variable = self.arm_group.get_current_pose()
        print (variable.pose)
        rospy.sleep(1)

    # functions required for navigation to goal poses

    def active_cb(self):
        rospy.loginfo("Goal pose being processed")

    def feedback_cb(self, feedback):
        # rospy.loginfo("Current location: "+str(feedback))
        pass


    def done_cb(self, status, result):
        if status == 3:
            rospy.loginfo("Goal reached")
        if status == 2 or status == 8:
            rospy.loginfo("Goal cancelled")
        if status == 4:
            rospy.loginfo("Goal aborted")


    def moveToPose(self, x, y, z, qx, qy, qz ,qw):
        # 7th intermediate pose
        global navclient
        global finished
        navclient = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        navclient.wait_for_server()


        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.position.z = z
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        navclient.send_goal(goal, self.done_cb, self.active_cb, self.feedback_cb)
        finished = navclient.wait_for_result(rospy.Duration(30,0))

        navclient.wait_for_server()


if __name__ == '__main__':

    # Initialize
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('take_photo', anonymous=False)

    
    
    camera = TakePhoto()

    while not rospy.is_shutdown():
        rospy.sleep(0.1)
        rospy.spin()
        moveit_commander.roscpp_shutdown()

    camera.stop

