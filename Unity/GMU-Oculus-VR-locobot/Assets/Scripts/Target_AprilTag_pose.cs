using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using static RosSharp.TransformExtensions; // for transformation (Ros2Unity)
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using geo_msgs = RosSharp.RosBridgeClient.MessageTypes.Geometry;

using UnityEngine;

public class Target_AprilTag_pose : MonoBehaviour
{
    RosSocket rosSocket;
    public GameObject parent;
    private string RosBridgeServerUrl;

    private Vector3 position_unity;
    private Vector3 position_ros;
    private Quaternion rotation_ros;
    private Quaternion rotation_unity;
    private string ca_id;
    private bool isMessageReceived;
   
    void Start()
    {
        RosBridgeServerUrl = GameObject.FindGameObjectWithTag("UpdateIP").GetComponent<UpdateRosIP>().getRosIP();

        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        Debug.Log("Established connection with ros");
        ca_id = rosSocket.Subscribe<geo_msgs.Pose>("/tagpose", Sub_AprilTag_pose);
    }

    private void Sub_AprilTag_pose(geo_msgs.Pose message)
    {
        //Debug.Log("received");
        isMessageReceived = true;
        position_ros.x = (float)message.position.x;
        position_ros.y = (float)message.position.y;
        position_ros.z = (float)message.position.z;

        //Debug.Log("position_ros " + position_ros);
        position_unity = position_ros.Ros2Unity();
        position_unity.y = position_unity.y - (float) 0.05;
        //Debug.Log("position_unity " + position_unity);

        rotation_ros.x = (float)message.orientation.x;
        rotation_ros.y = (float)message.orientation.y;
        rotation_ros.z = (float)message.orientation.z;
        rotation_ros.w = (float)message.orientation.w;
        rotation_unity = rotation_ros.Ros2Unity();

    }
    // Update is called once per frame
    void Update()
    {
        if (isMessageReceived)
            parent.transform.localPosition = position_unity;
            parent.transform.localRotation = rotation_unity;
    }
}
