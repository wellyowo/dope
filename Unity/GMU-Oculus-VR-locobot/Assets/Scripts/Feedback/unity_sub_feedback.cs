using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using static RosSharp.TransformExtensions; // for transformation (Ros2Unity)
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using geo_msgs = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Photon.Pun;
using UnityEngine;
using System;
using System.IO;
using System.Text;
using System.Threading;

public class unity_sub_feedback : MonoBehaviour
{
    public GameObject feedbackInput, controllerInput;
    private PhotonView feedback;
    private bool gripper_state_MessageReceived, joint_state_MessageReceived,image_MessageReceived; //detect
    private bool pri_MessageReceived, sec_MessageReceived, joint_MessageReceived; //immediately

    RosSocket rosSocket;

    private string RosBridgeServerUrl; //IP address
    private string gripper_state_feedback_id, joint_state_feedback_id, camera_feedback_id, camera_utc_feedback_id; //detect
    private string pri_feedback_id, sec_feedback_id, joint_feedback_id; //immediately

    private string test_unity_clock_id, pri_unity_clock_id, sec_unity_clock_id, gra_unity_clock_id; //clock

    private bool cur_gripper_state, pre_gripper_state, cur_joint_state, pre_joint_state;

    private bool gripper_closed, joint_inposition;
    private float time;
    private float updateCount = 0, pre_updateCount = 0;

    private string filePath;
    const string format = "yyyy'-'MM'-'dd' 'HH':'mm':'ss'.'ffffff''";
    private DateTime datetime_pri_trigger, datetime_sec_triger, datetime_gra_trigger;
    private string time_pri_trigger, time_sec_triger, time_gra_trigger; //triger feedback - time
    private bool trigger_pri, trigger_sec, trigger_gra; //triger feedback - bool
    public bool gripper_trigger = false, joint_trigger = false, image_trigger = false; //detect

    void Start()
    {
        string name = "Sub";
        string utc = DateTime.UtcNow.ToString(format);
        string add_name_utc = utc + "," + name;
        Debug.Log("start_time" + add_name_utc);

        //save file stat
        filePath = Application.dataPath + "/data.csv";
        FileStream fs = new FileStream(filePath, FileMode.Create, FileAccess.Write);
        System.IO.FileInfo FileAttribute = new FileInfo(filePath);
        FileAttribute.Attributes = FileAttributes.Normal;
        StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.Default);
        sw.Write("robot_side - tigger_name" + "," + "time" + "\n");
        sw.Close();
        //end file initial

        feedback = feedbackInput.GetComponent<PhotonView>();

        //
        RosBridgeServerUrl = GameObject.FindGameObjectWithTag("UpdateIP").GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        //Debug.Log("Sub_feedback_sub,Established connection with ros");

        //subscibler
        //gripper_state_feedback_id = rosSocket.Subscribe<std_msgs.Bool>("/gripper_state_feedback", gripper_state_callback); //detect
        //joint_state_feedback_id = rosSocket.Subscribe<std_msgs.Bool>("/joint_state_feedback", joint_state_callback); //detect

        pri_feedback_id = rosSocket.Subscribe<std_msgs.Bool>("/pri_feedback", pri_callback); //immediately
        sec_feedback_id = rosSocket.Subscribe<std_msgs.Bool>("/sec_feedback", sec_callback); //immediately
        joint_feedback_id = rosSocket.Subscribe<std_msgs.Bool>("/joint_feedback", joint_callback); //immediately

        camera_feedback_id = rosSocket.Subscribe<sensor_msgs.CompressedImage>("/snap/compressed", camera_callback);
        camera_utc_feedback_id = rosSocket.Subscribe<std_msgs.String>("/snap_clock", camera_utc_callback);

    }

    private void pri_callback(std_msgs.Bool msg)
    {
        pri_MessageReceived = true;
    }

    private void sec_callback(std_msgs.Bool msg)
    {
        sec_MessageReceived = true;
    }

    private void joint_callback(std_msgs.Bool msg)
    {
        joint_MessageReceived = true;
    }

    private void savetofile(String tigger_name, String time)
    {
        //Debug.Log(tigger);
        //Debug.Log(time);
        //Debug.Log("Save");

        FileStream fs = new FileStream(filePath, FileMode.Append, FileAccess.Write);
        StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.Default);

        sw.Write(tigger_name + "," + time + "\n");
        sw.Close();
    }

    private void gripper_state_callback(std_msgs.Bool msg)
    {
        gripper_state_MessageReceived = true;

        cur_gripper_state = msg.data; //current

        if (cur_gripper_state == true)
        {
            gripper_closed = true;
        }
        else
        {
            gripper_closed = false;
        }

        if (pre_gripper_state != cur_gripper_state)
        {
            gripper_trigger = true;
        }

        else
        {
            gripper_trigger = false;
        }
        pre_gripper_state = cur_gripper_state;
    }

    private void joint_state_callback(std_msgs.Bool msg)
    {
        joint_state_MessageReceived = true;
        cur_joint_state = msg.data; //current

        if (cur_joint_state == true)
        {
            joint_inposition = true;
        }
        else
        {
            joint_inposition = false;
        }

        if (pre_joint_state != cur_joint_state)
        {
            joint_trigger = true;
        }

        else
        {
            joint_trigger = false;
        }
        pre_joint_state = cur_joint_state;
    }

    private void camera_callback(sensor_msgs.CompressedImage msg)
    {
        Debug.Log("ros_camera" + DateTime.UtcNow.ToString(format));
    }

    private void camera_utc_callback(std_msgs.String msg)
    {
        image_MessageReceived = true;
        savetofile("ros_camera_utc_callback_ros", msg.data);
        savetofile("ros_camera_utc_callback_unity", DateTime.UtcNow.ToString(format));
        //Debug.Log("ros_camera_utc_callback_ros" + msg.data);
        //Debug.Log("ros_camera_utc_callback_unity" + DateTime.UtcNow.ToString(format));
    }


    // Update is called once per frame
    void Update()
    {
        savetofile("Sub", DateTime.UtcNow.ToString(format));

        trigger_pri = this.GetComponent<unity_pub_feedback>().pri_fb_trigger;
        trigger_sec = this.GetComponent<unity_pub_feedback>().sec_fb_trigger;
        trigger_gra = this.GetComponent<unity_pub_feedback>().joint_fb_trigger;

        if (pri_MessageReceived == true & trigger_pri == true)
        {
            savetofile("robot_side - pri_fb_trigger", DateTime.UtcNow.ToString(format));
            feedback.RPC("getrospri_Network", RpcTarget.All, trigger_pri);
        }
        if (sec_MessageReceived == true & trigger_sec == true)
        {
            savetofile("robot_side - sec_fb_trigger", DateTime.UtcNow.ToString(format));
            feedback.RPC("getrossec_Network", RpcTarget.All, trigger_sec);
        }
        if (joint_MessageReceived == true & trigger_gra ==true)
        {
            savetofile("robot_side - joint_fb_trigger", DateTime.UtcNow.ToString(format));
            feedback.RPC("getrosjoint_Network", RpcTarget.All, trigger_gra);
        }


        //if (gripper_state_MessageReceived == true)
        //    {
        //        if (gripper_trigger == true)
        //        {
        //            savetofile("robot_side - gripper_trigger_from_ros", DateTime.UtcNow.ToString(format));
        //            //Debug.Log("gripper_trigger" + DateTime.UtcNow.ToString(format));
        //            feedback.RPC("getrosgt_Network", RpcTarget.All, gripper_trigger);
        //            //Debug.Log("1");
        //        }
        //    }

        //    if (joint_state_MessageReceived == true)
        //    {
        //        //
        //        if (joint_trigger == true)
        //        {
        //            savetofile("robot_side - joint_trigger_from_ros", DateTime.UtcNow.ToString(format));
        //            //Debug.Log("joint_trigger" + DateTime.UtcNow.ToString(format));
        //            feedback.RPC("getrosjt_Network", RpcTarget.All, joint_trigger);
        //            //Debug.Log("joint_state change");
        //        }

        //    }

        //if(image_MessageReceived == true)
        //{
        //    image_trigger = true;
        //    Debug.Log("having iamge");
        //    feedback.RPC("getrosim_Network", RpcTarget.All, image_trigger);
        //}
        //else
        //{
        //    image_trigger = false;
        //    feedback.RPC("getrosim_Network", RpcTarget.All, false);
        //}
        //image_MessageReceived = false; //Only listen once
        //image_trigger = false;
    }


    }

