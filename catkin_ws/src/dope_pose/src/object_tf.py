#!/usr/bin/env python3

import rospy
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped

class object_trans():
    def __init__(self, dope_sub_name, target_frame, source_frame):
        self.pose_sub = rospy.Subscriber(dope_sub_name, PoseStamped, self.pose_cb, queue_size = 10)
        self.pose_pub = rospy.Publisher(dope_sub_name + '_baselink', PoseStamped, queue_size = 10)
        self.target_frame = target_frame
        self.source_frame = source_frame
        self.buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.buffer)
        
    
    def pose_cb(self, msg):
        try: 
            transform = self.buffer.lookup_transform(self.target_frame, self.source_frame, rospy.Time(0))
            pose_transformed = tf2_geometry_msgs.do_transform_pose(msg, transform)
            self.pose_pub.publish(pose_transformed)
            print("new pose published")
        except:
            print('something wrong with tf')
            pass

        
        
if __name__ == "__main__":
    rospy.init_node("dope_pose_transform", anonymous=False)

    pose_butter = object_trans("/dope/pose_Butter", 'base_link', 'camera_left_color_optical_frame')
    pose_sugar = object_trans("/dope/pose_sugar", "base_link", 'camera_left_color_optical_frame')    
    rospy.spin()
