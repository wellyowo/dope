using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using geo_msgs = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using UnityEngine;
using UnityEngine.XR;
using System;

public class ControllerPublisher : MonoBehaviour
    {
        RosSocket rosSocket;
        string publication_test; //Pub_test
        string primary_button;
        string scondary_button;
        string grip;
        string trigger;
        string joystick;

        //VR Device
        public InputDeviceCharacteristics controllerCharacteristics;
        private InputDevice targetDevices; //what is your vr contoller input

        public string FrameId = "Unity";
        public GameObject IP;
        public GameObject Controller;

        public DateTime pri_trigger_time, sec_triger_time;  //triger feedback - time
        public bool pri_trigger = false, sec_trigger = false; //triger feedback - bool

        private string RosBridgeServerUrl; //IP address
        private bool primaryButtonValue, secondaryButtonValue;

    void Start()
    {
        //VR Device
        /*List<InputDevice> devices = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(controllerCharacteristics, devices);
            if (devices.Count > 0)
            {
            targetDevices = devices[0];
            Debug.Log(devices[0]);
            }*/
           

        //RosSocket
        RosBridgeServerUrl = IP.GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        Debug.Log("Established connection with ros");

        //Topic name
        publication_test = rosSocket.Advertise<std_msgs.String>("publication_test"); //Pub_test
        primary_button = rosSocket.Advertise<std_msgs.Bool>("/vr/right/primarybutton");
        scondary_button = rosSocket.Advertise<std_msgs.Bool>("/vr/right/secondarybutton");
        grip = rosSocket.Advertise<std_msgs.Float32>("vr/grip");
        trigger = rosSocket.Advertise<std_msgs.Float32>("vr/trigger");
        joystick = rosSocket.Advertise<sensor_msgs.Joy>("vr/joystick");

    }


    void Update()
     {
        /*//------------------Pub_test------------------------------//
        std_msgs.String message_test = new std_msgs.String
        { 
             data = "Message sent from unity"
        };
        rosSocket.Publish(publication_test, message_test);
        //Debug.Log("A message was sent to ROS");*/
        //------------------Pub_test------------------------------//

        primaryButtonValue = Controller.GetComponent<ControllersManager>().getPrimaryButton();
        if (primaryButtonValue == true)
        {
            std_msgs.Bool message_p = new std_msgs.Bool
            {
            data = primaryButtonValue
            };

            rosSocket.Publish(primary_button, message_p);
            pri_trigger_time = DateTime.UtcNow;
            //pri_trigger = true;
        }
        //else
        //{
        //    pri_trigger = false;
        //}

        secondaryButtonValue = Controller.GetComponent<ControllersManager>().getSecondaryButton();
        if (secondaryButtonValue)
        {
            std_msgs.Bool message_s = new std_msgs.Bool
            {
                data = secondaryButtonValue
            };
            rosSocket.Publish(scondary_button, message_s);
            sec_triger_time = DateTime.UtcNow;
            //sec_trigger = true;
        }

        //else
        //{
        //    sec_trigger = false;
        //}

    }
}
    


