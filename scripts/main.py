#!/usr/bin/env python
# -*- coding: utf-8 -*-
#respect shunsuke-f
import rospy

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib_msgs
from actionlib_msgs.msg import GoalStatus

from geometry_msgs.msg import Twist

import tf
# Ref: https://hotblackrobotics.github.io/en/blog/2018/01/29/action-client-py/

import math
from std_msgs.msg import Int32MultiArray

from std_msgs.msg import String,Bool,Int32
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2


class NaviBot():
    def __init__(self):

        # velocity publisher
        self.vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
        
        # debug用
        self.diff_pub = rospy.Publisher('diff_degree', Int32, queue_size=1)
        
        # 敵の緑の的の重心座標を受け取る
        self.enemy_green_center_sub = rospy.Subscriber('enemy_green_center', Int32MultiArray, self.enemyGreenCenterCallback)

        self.cx = 0
        self.cy = 0

        # amclの推定位置が入る
        self.pose_x = 0
        self.pose_y = 0
        self.th = 0

    def isEnemyPointsCallback(self, msg):
        self.is_enemy_points = msg.data

    def enemyDirectionCallback(self, msg):
        self.enemy_direction_deg = msg.data

    def enemyGreenCenterCallback(self, msg):
        self.cx = msg.data[0]
        self.cy = msg.data[1]
        #        print(self.cx, self.cy)
        #        rospy.loginfo(self.cx, self.cy)



    def setGoal(self,x,y,yaw):
        self.client.wait_for_server()

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y

        # Euler to Quartanion
        q=tf.transformations.quaternion_from_euler(0,0,yaw)        
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.send_goal(goal)
        wait = self.client.wait_for_result()
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            return self.client.get_result()        


    def strategy(self):
        r = rospy.Rate(5) # change speed 5fps
        #0
        self.str
        self.setGoal(-0.8,0,3.1415/2)
        #a
        self.str
        self.setGoal(-0.8,0.5,3.1415/2)
        self.str
        self.setGoal(-0.8,0.5,0)
        #l
        self.str
        self.setGoal(-0.5,0,0)
        self.str
        self.setGoal(-0.30,0.30,3.1415/4)
        #b
        self.str
        self.setGoal(-0.25,0.5,3.1415)
        self.str
        self.setGoal(-0.25,0.7,0)
        #c
        self.str
        self.setGoal(0,0.7,-3.1415/2)
        self.str
        self.setGoal(0.25,0.7,0)
        #d
        self.str
        self.setGoal(0.25,0.5,0)
        #f
        self.str
        self.setGoal(0.5,0,3.1415)
        #e
        self.str
        self.setGoal(0.8,0.5,3.1415)
        self.str
        self.setGoal(0.8,0,-3.1415/2)
        #g
        self.str
        self.setGoal(0.8,-0.5,3.1415)
        self.str
        self.setGoal(0.3,-0.3,-3.1415/2)
        #h
        self.str
        self.setGoal(0.25,-0.5,0)
        self.str
        self.setGoal(0.25,-0.8,3.1415)
        #i
        self.str
        self.setGoal(0,-0.8,3.1415/2)
        self.str
        self.setGoal(-0.25,-0.8,3.1415/2)
        #j
        self.str
        self.setGoal(-0.25,-0.5,3.1415)
        self.str
        self.setGoal(-0.3,-0.3,3.1415)
        self.str
        self.setGoal(-0.8,-0.3,3.1415)
        #k
        self.str
        self.setGoal(-0.8,-0.5,0)


    def str(self):
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            # カメラ画像内に敵がいた場合
            if self.cx != 0 and self.cy != 0:
                self.client.cancel_goal()
                
                # 真ん中を向くように方向転換
                diff_pix = self.cx - 320
                anglular_z = 0
                if abs(diff_pix) < 320 and diff_pix > 20:
                    anglular_z = -0.3
                elif abs(diff_pix) < 320 and diff_pix < -20:
                    glular_z = 0.3
                else:
                    anglular_z = 0.0
                    
                twist = Twist()
                twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0
                twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = anglular_z
                self.vel_pub.publish(twist)

            else:
                rospy.loginfo(self.client.get_state())       
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('Samplerun')
    bot = NaviBot()
    while True:
        bot.strategy()
