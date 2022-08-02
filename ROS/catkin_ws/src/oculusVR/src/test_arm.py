#!/usr/bin/env python
import rospy
import tf
import math
import numpy as np
import time
from std_msgs.msg import Bool, Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import PoseStamped, Pose
from std_srvs.srv import Trigger, TriggerResponse
from sensor_msgs.msg import JointState
from pyrobot import Robot

class subArm():

    def __init__(self):
        self.robot = Robot('locobot')
        self.sub_primary = rospy.Subscriber("/vr/right/primarybutton", Bool, self.primarybutton_callback,queue_size=3)	#gripper open
        self.sub_second = rospy.Subscriber("/vr/right/secondarybutton", Bool, self.secondarybutton_callback,queue_size=1)
        self.sun_jointstate = rospy.Subscriber("/joint_states_test", JointState, self.jointstate_callback, queue_size=1)

	#gripper close
        # ros service
        start = rospy.Service("/initial", Trigger, self.initial)

        self.robot.arm.go_home()

    def initial(self, req):

        res = TriggerResponse()

        try:
	    self.robot.arm.go_home()
            res.success = True
        except (rospy.ServiceException, rospy.ROSException) as e:
            res.success = False
            print("Service call failed: %s"%e)
        
        return res

    def gripper_close(self):
        print("close")
        self.robot.gripper.close()

    def gripper_open(self):
        print("open")
        self.robot.gripper.open()

    def primarybutton_callback(self, msg_close):
        p_close = msg_close.data
        #print("primary", p_close)
        if (p_close == True):
            self.gripper_close()
    
    def secondarybutton_callback(self, msg_open):
        p_open=msg_open.data
        #print("second",  p_open)
        if (p_open == True):
            self.gripper_open()

    def jointstate_callback(self,joint_data):
        self.joint = JointState()
        self.joint = joint_data
        print(joint_data.position)

        _joint = self.joint
        #target_joints = [[0.0, 0.6062774658203125, -0.9323844313621521, 0.3261069655418396, 0.0]]
        joint_1 = _joint.position[2]
        joint_2 = _joint.position[3]
        joint_3 = _joint.position[4]
        joint_4 = _joint.position[5]
        joint_5 = _joint.position[6]
        target_joints = [[joint_1, joint_2, joint_3, joint_4, joint_5]]
        
        for joint in target_joints:
            self.robot.arm.set_joint_positions(joint, plan=False)
            #time.sleep(1)
        


    

if __name__ == '__main__':
    rospy.init_node('testarm')
    test = subArm()
    rospy.spin()
