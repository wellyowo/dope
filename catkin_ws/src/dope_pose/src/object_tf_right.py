#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
import numpy as np
from geometry_msgs.msg import PoseStamped

class object_trans():
    def __init__(self, dope_sub_name, target_frame, source_frame):
        self.pose_sub = rospy.Subscriber(dope_sub_name, PoseStamped, self.pose_cb, queue_size = 10)
        self.pose_pub = rospy.Publisher(dope_sub_name + '_baselink', PoseStamped, queue_size = 10)
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        self.pos_x_list = np.zeros(10)
        self.pos_y_list = np.zeros(10)
        self.pos_z_list = np.zeros(10)
        self.ori_x_list = np.zeros(10)
        self.ori_y_list = np.zeros(10)
        self.ori_z_list = np.zeros(10)
        self.ori_w_list = np.zeros(10)
        self.i = 0
        self.pose_result = PoseStamped()
        
    
    def pose_cb(self, msg):
        try: 
            transform = self.buffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(0))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
            self.pos_x_list[self.i] = pose_transformed.pose.position.x
            self.pos_y_list[self.i] = pose_transformed.pose.position.y
            self.pos_z_list[self.i] = pose_transformed.pose.position.z
            self.ori_x_list[self.i] = pose_transformed.pose.orientation.x
            self.ori_y_list[self.i] = pose_transformed.pose.orientation.y
            self.ori_z_list[self.i] = pose_transformed.pose.orientation.z
            self.ori_w_list[self.i] = pose_transformed.pose.orientation.w
            if self.i == 9:
                self.i = 0
            else:
                self.i += 1
            self.pose_result.pose.position.x, self.pose_result.pose.position.y, self.pose_result.pose.position.z, self.pose_result.pose.orientation.x, self.pose_result.pose.orientation.y, self.pose_result.pose.orientation.z, self.pose_result.pose.orientation.w = self.cal_avg()
            self.pose_pub.publish(self.pose_result)
            print("new pose published")
        except:
            print('something wrong with tf')
            pass
    
    def cal_avg(self):
        pos_x = np.mean(self.pos_x_list)
        pos_y = np.mean(self.pos_y_list)
        pos_z = np.mean(self.pos_z_list)
        ori_x = np.mean(self.ori_x_list)
        ori_y = np.mean(self.ori_y_list)
        ori_z = np.mean(self.ori_z_list)
        ori_w = np.mean(self.ori_w_list)
        return pos_x, pos_y, pos_z, ori_x, ori_y, ori_z ,ori_w
        


        
        
if __name__ == "__main__":
    rospy.init_node("dope_pose_transform", anonymous=False)

    pose_butter = object_trans("/dope/pose_Butter", 'map', 'camera_right_color_optical_frame')
    pose_sugar = object_trans("/dope/pose_sugar", "map", 'camera_right_color_optical_frame')    
    rospy.spin()
