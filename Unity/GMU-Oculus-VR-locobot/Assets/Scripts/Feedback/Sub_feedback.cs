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

public class Sub_feedback : MonoBehaviour
{
    private PhotonView reciever, feedback;
    public GameObject controllerInput, feedbackInput;

    private bool isMessageReceived;

    RosSocket rosSocket;

    private string RosBridgeServerUrl; //IP address
    private string gripper_state_feedback_id, joint_state_feedback_id, camera_feedback_id, camera_utc_feedback_id; //
    private string test_unity_clock_id, pri_unity_clock_id, sec_unity_clock_id, gra_unity_clock_id; //clock
    private bool cur_gripper_state, pre_gripper_state, cur_joint_state, pre_joint_state;
    private bool gripper_closed, joint_inposition;
    private float time;
    private float updateCount = 0, pre_updateCount = 0;
    public bool gripper_trigger = false, joint_trigger = false;
    private string filePath;
    const string format = "yyyy'-'MM'-'dd' 'HH':'mm':'ss'.'ffffff''";
    private DateTime datetime_pri_trigger, datetime_sec_triger, datetime_gra_trigger;
    private string time_pri_trigger, time_sec_triger, time_gra_trigger; //triger feedback - time

    private bool trigger_pri, trigger_sec, trigger_gra; //triger feedback - bool


    void Start()
    {
        string name = "Hello";
        string utc = DateTime.UtcNow.ToString(format);
        string add_name_utc = utc + "," + name;
        Debug.Log("start_time" + add_name_utc);

        //save file stat
        filePath = Application.dataPath + "/data.csv";
        FileStream fs = new FileStream(filePath, FileMode.Create, FileAccess.Write);
        System.IO.FileInfo FileAttribute = new FileInfo(filePath);
        FileAttribute.Attributes = FileAttributes.Normal;
        StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.Default);
        sw.Write("tigger_name" + "," + "time" + "\n");
        sw.Close();
        //end file initial
        //
        reciever = controllerInput.GetComponent<PhotonView>();
        feedback = this.GetComponent<PhotonView>();

        //
        RosBridgeServerUrl = GameObject.FindGameObjectWithTag("UpdateIP").GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        Debug.Log("Sub_feedback,Established connection with ros");
        
        //publisher
        test_unity_clock_id = rosSocket.Advertise<std_msgs.String>("/test_unity_clock");
        pri_unity_clock_id = rosSocket.Advertise<std_msgs.String>("/pri/unity_clock");
        sec_unity_clock_id = rosSocket.Advertise<std_msgs.String>("/sec/unity_clock");
        gra_unity_clock_id = rosSocket.Advertise<std_msgs.String>("/gra/unity_clock");

        //subscibler
        gripper_state_feedback_id = rosSocket.Subscribe<std_msgs.Bool>("/gripper_state_feedback", gripper_state_callback);
        joint_state_feedback_id = rosSocket.Subscribe<std_msgs.Bool>("/joint_state_feedback", joint_state_callback);
        camera_feedback_id = rosSocket.Subscribe<sensor_msgs.CompressedImage>("/snap/compressed", camera_callback);
        camera_utc_feedback_id = rosSocket.Subscribe<std_msgs.String>("/snap_clock", camera_utc_callback);

    }


    private void savetofile(String tigger_name, String time)
    {
        //Debug.Log(tigger);
        //Debug.Log(time);
        Debug.Log("Enter");

        FileStream fs = new FileStream(filePath, FileMode.Append, FileAccess.Write);
        StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.Default);

        sw.Write(tigger_name + "," + time + "\n");
        sw.Close();
    }

    private void gripper_state_callback(std_msgs.Bool msg)
    {
        isMessageReceived = true;

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
        savetofile("ros_camera_utc_callback_ros", msg.data);
        savetofile("ros_camera_utc_callback_unity", DateTime.UtcNow.ToString(format));
        //Debug.Log("ros_camera_utc_callback_ros" + msg.data);
        //Debug.Log("ros_camera_utc_callback_unity" + DateTime.UtcNow.ToString(format));
    }


    // Update is called once per frame
    void Update()
    {
        updateCount += 1;
        //test
        //string name = "Hello";
        //string utc = DateTime.UtcNow.ToString(format);
        //string add_name_utc = utc + "," + name;
        //std_msgs.String test_clock = new std_msgs.String
        //{
        //    data = add_name_utc
        //};
        //rosSocket.Publish(test_unity_clock_id, test_clock);

        //Debug.Log("Enter feedback code");
        //trigger_pri = this.GetComponent<ControllerPublisher>().pri_trigger;
        //trigger_sec = this.GetComponent<ControllerPublisher>().sec_trigger;
        //trigger_gra = this.GetComponent<Drawing>().gra_trigger;


        trigger_pri = controllerInput.GetComponent<ControllersManager>().getP_T();
        trigger_sec = controllerInput.GetComponent<ControllersManager>().getS_T();
        trigger_gra = controllerInput.GetComponent<ControllersManager>().getG_T();


        if (trigger_pri == true)
            {
                Debug.Log("Pri_Pressed");
                //Debug.Log("start open gripper" + DateTime.UtcNow.ToString(format));
                datetime_pri_trigger = this.GetComponent<ControllerPublisher>().pri_trigger_time;
                time_pri_trigger = datetime_pri_trigger.ToString(format) + "," + "pri_trigger";
                std_msgs.String pri_clock = new std_msgs.String
                {
                    data = time_pri_trigger
                };
                rosSocket.Publish(pri_unity_clock_id, pri_clock);
                savetofile("start open gripper", DateTime.UtcNow.ToString(format));
            }
        
        if (trigger_sec == true)
            {
                Debug.Log("Sec_Pressed");
                //Debug.Log("start close gripper" + DateTime.UtcNow.ToString(format));
                datetime_sec_triger = this.GetComponent<ControllerPublisher>().sec_triger_time;
                time_sec_triger = datetime_sec_triger.ToString(format) + "," + "sec_trigger";
                std_msgs.String sec_clock = new std_msgs.String
                {
                    data = time_sec_triger
                };
                rosSocket.Publish(sec_unity_clock_id, sec_clock);
                savetofile("start close gripper", DateTime.UtcNow.ToString(format));
            }

            if (trigger_gra == true)
            {
                Debug.Log("Gra_Pressed");
                //Debug.Log("start publish joint" + DateTime.UtcNow.ToString(format));
                datetime_gra_trigger = this.GetComponent<Drawing>().gra_trigger_time;
                time_gra_trigger = datetime_gra_trigger.ToString(format) + "," + "gra_trigger";
                std_msgs.String gra_clock = new std_msgs.String
                {
                    data = time_gra_trigger
                };
                rosSocket.Publish(gra_unity_clock_id, gra_clock);
                savetofile("start publish joint", DateTime.UtcNow.ToString(format));
            }

            if (gripper_trigger == true)
            {
                savetofile("gripper_trigger_from_ros", DateTime.UtcNow.ToString(format));
                Debug.Log("gripper_trigger" + DateTime.UtcNow.ToString(format));
            }

            if (joint_trigger == true)
            {
                savetofile("joint_trigger_from_ros", DateTime.UtcNow.ToString(format));
                Debug.Log("joint_trigger" + DateTime.UtcNow.ToString(format));
            }

        }

}
