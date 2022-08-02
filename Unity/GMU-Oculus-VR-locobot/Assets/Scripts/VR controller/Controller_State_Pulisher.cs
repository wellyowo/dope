using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using geo_msgs = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using static RosSharp.TransformExtensions; 
using UnityEngine;
using UnityEngine.XR;

public class Controller_State_Pulisher : MonoBehaviour
{
    RosSocket rosSocket;
    private string RosBridgeServerUrl; //IP address

    string controller_position;
    string primary_button;
    string scondary_button;

    //VR Device
    public InputDeviceCharacteristics controllerCharacteristics;
    private InputDevice targetDevices; //what is your vr contoller input

    public string FrameId = "Unity";

    //VR Controller - Device
    private Vector3 position_device;
    //VR Controller - Unity Coordinate
    public GameObject locobot_arm;
    public GameObject vr_controller;
    //Unity corrdinate
    private Vector3 position_unity;
    private Vector3 locobot_arm_position;
    private Vector3 translate;
    private Quaternion rotation_unity;
    //Ros corrdinate
    private Vector3 position_ros;
    private Quaternion rotation_ros;

    private int posCount = 0, i = 0;
    private float timer = 0.0f;
    private List<geo_msgs.Pose> posesList = new List<geo_msgs.Pose>();
    private geo_msgs.Pose gripperOpenPlace;
    private geo_msgs.Pose gripperClosePlace;
    private bool isConfirmed, isOpen, isClose;

    // Start is called before the first frame update
    void Start()
    {
        //VR Device
        List<InputDevice> devices = new List<InputDevice>();
        InputDevices.GetDevicesWithCharacteristics(controllerCharacteristics, devices);
        if (devices.Count > 0)
        {
            targetDevices = devices[0];
            Debug.Log(devices[0]);
        }

        //RosSocket
        RosBridgeServerUrl = GameObject.FindGameObjectWithTag("UpdateIP").GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        Debug.Log("Established connection with ros");

        //Topic name
        controller_position = rosSocket.Advertise<geo_msgs.Pose>("vr/controller_position");
        primary_button = rosSocket.Advertise<std_msgs.Bool>("vr/primarybutton");
        scondary_button = rosSocket.Advertise<std_msgs.Bool>("vr/secondarybutton");

    }

    // Update is called once per frame
    void Update()
    {
        //VR Controller - Device
        targetDevices.TryGetFeatureValue(CommonUsages.grip, out float gripValue);
        targetDevices.TryGetFeatureValue(CommonUsages.trigger, out float triggerValue);
        targetDevices.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryButtonValue);
        targetDevices.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryButtonValue);

        //targetDevices.TryGetFeatureValue(CommonUsages.devicePosition, out Vector3 device_position);
        //position_device = device_position;

        //VR Controller - Unity Coordinate
        translate = vr_controller.transform.position;
            //Debug.Log("traslation_x" + translate.x);
            //Debug.Log("traslation_y" + translate.y);
            //Debug.Log("traslation_z" + translate.z);
        rotation_unity = vr_controller.transform.rotation;
            //Debug.Log("rotation_x" + rotation_unity.x);
            //Debug.Log("rotation_y" + rotation_unity.y);
            //Debug.Log("rotation_z" + rotation_unity.z);
            //Debug.Log("rotation_w" + rotation_unity.w);

        //Change coordiante from unity to ros
        position_ros = translate.Unity2Ros();
        rotation_ros = rotation_unity.Unity2Ros();
        //Debug.Log("Device_position " + position_device);
        //Debug.Log("Device_position_ros " + position_ros);

        timer += Time.deltaTime;

        //Push the Trigger, then publish the controller_position from unity to ros
        if (gripValue > 0.7f)
        {
            //Debug.Log("gripValue " + gripValue);
            geo_msgs.Point temp_position = new geo_msgs.Point
            {
                x = position_ros.x,
                y = position_ros.y,
                z = position_ros.z
            };

            geo_msgs.Quaternion temp_quaternion = new geo_msgs.Quaternion
            {
                x = rotation_ros.x,
                y = rotation_ros.y,
                z = rotation_ros.z,
                w = rotation_ros.w
            };

            geo_msgs.Pose message_d_position = new geo_msgs.Pose
            {
                position = temp_position,
                orientation = temp_quaternion
            };

            

            if (posCount % 5 == 0)
            {
                if (primaryButtonValue || secondaryButtonValue)
                {
                    if (primaryButtonValue)
                    {
                        gripperClosePlace = message_d_position;
                        isClose = true;
                        //Debug.Log("Primary @: " + gripperClosePlace.position.x);
                    }

                    else
                    {
                        gripperOpenPlace = message_d_position;
                        isOpen = true;
                        // Debug.Log("Secondary @: " + gripperOpenPlace.position.x);
                    }
                }
                //rosSocket.Publish(controller_position, message_d_position);
                posesList.Add(message_d_position);
                Debug.Log("#" + (posCount) + ": " + message_d_position.position.x);
            }
            posCount++;

            //Debug.Log("Publish");
        }

        if (triggerValue >= 0.8)
        {
            isConfirmed = true;
        }

        if (timer >= 2f)
        {
            if (posesList.Count > i)
            {
                if (isConfirmed)
                {
                    rosSocket.Publish(controller_position, posesList[i]);
                    //Debug.Log("Publishing: "+ posesList[i++]);
                    if (isClose)
                    {
                        Debug.Log("Pose: " + posesList[i].position.x);
                        Debug.Log("Gripper: " + gripperClosePlace.position.x);
                        if (posesList[i] == gripperClosePlace)
                        {
                            setPrimaryButton(isClose);
                            isClose = false;
                        }
                        //Debug.Log("Close: " + isClose);
                    }

                    if (isOpen)
                    {
                        if (posesList[i] == gripperOpenPlace)
                        {
                            setSecondaryButton(isOpen);
                            isOpen = false;
                        }
                    }
                    i++;
                }
                
            }
            else
            {
                isConfirmed = false;
            }
            timer = 0f;
        }




    }

    private void setPrimaryButton(bool primaryButtonValue)
    {
        //------------------Pub_Primary Buttom------------------------------//
        std_msgs.Bool message_p = new std_msgs.Bool
        {
            data = primaryButtonValue
        };
        rosSocket.Publish(primary_button, message_p);
        //   }
    }

    private void setSecondaryButton(bool secondaryButtonValue)
    {
        std_msgs.Bool message_s = new std_msgs.Bool
        {
            data = secondaryButtonValue
        };

        rosSocket.Publish(scondary_button, message_s);
    }
}


