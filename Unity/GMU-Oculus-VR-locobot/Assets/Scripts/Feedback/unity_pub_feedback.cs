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

public class unity_pub_feedback : MonoBehaviour
{
    private PhotonView controller_reciever;
    public GameObject controllerInput;
    private bool isMessageReceived;

    RosSocket rosSocket;

    private string RosBridgeServerUrl; //IP address
    private string gripper_state_feedback_id, joint_state_feedback_id, camera_feedback_id, camera_utc_feedback_id; //
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


    public bool pri_fb_trigger = false, sec_fb_trigger = false, joint_fb_trigger = false; //immediately


    void Start()
    {
        string name = "Pub";
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

        //PUN2
        controller_reciever = controllerInput.GetComponent<PhotonView>();

        RosBridgeServerUrl = GameObject.FindGameObjectWithTag("UpdateIP").GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        //Debug.Log("Sub_feedback_pub,Established connection with ros");
        
        //publisher
        test_unity_clock_id = rosSocket.Advertise<std_msgs.String>("/test_unity_clock");
        pri_unity_clock_id = rosSocket.Advertise<std_msgs.String>("/pri/unity_clock");
        sec_unity_clock_id = rosSocket.Advertise<std_msgs.String>("/sec/unity_clock");
        gra_unity_clock_id = rosSocket.Advertise<std_msgs.String>("/gra/unity_clock");
    }


    private void savetofile(String tigger_name, String time)
    {
        //Debug.Log(tigger);
        //Debug.Log(time);
        Debug.Log("Save");

        FileStream fs = new FileStream(filePath, FileMode.Append, FileAccess.Write);
        StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.Default);

        sw.Write(tigger_name + "," + time + "\n");
        sw.Close();
    }

    // Update is called once per frame
    void Update()
    {
        trigger_pri = controllerInput.GetComponent<ControllersManager>().getP_T();
        trigger_sec = controllerInput.GetComponent<ControllersManager>().getS_T();
        trigger_gra = controllerInput.GetComponent<ControllersManager>().getG_T();


        savetofile("Pub", DateTime.UtcNow.ToString(format));

        if (trigger_pri == true)
            {
                pri_fb_trigger = trigger_pri;
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
            else
            {
                pri_fb_trigger = false;
            }

        if (trigger_sec == true)
            {
                sec_fb_trigger = trigger_sec;
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
            else
            {
                sec_fb_trigger = false;
            }

        if (trigger_gra == true)
            {
                joint_fb_trigger = trigger_gra;
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
            else
            {
                joint_fb_trigger = false;
            }

        }

}
