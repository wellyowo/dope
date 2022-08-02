using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using static RosSharp.TransformExtensions; // for transformation (Ros2Unity)
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using geo_msgs = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Photon.Pun;
using UnityEngine;

public class Target_dope_pose : MonoBehaviour
{
    RosSocket rosSocket;
    private string RosBridgeServerUrl; //IP address
    //public GameObject IP;
    public GameObject parent;
    public Material transparentMaterial;
    public Material realMaterial;
    public GameObject texture;

    private Vector3 position_unity;
    private Vector3 position_ros;
    private Quaternion rotation_ros;
    private Quaternion rotation_unity;
    private string ca_id;
    private bool isMessageReceived = false, isStart = false, boolTrue, boolFalse;
    private PhotonView reciever;

    private float timerFirst = 0.0f;

    // Start is called before the first frame update
    void Start()
    {

        RosBridgeServerUrl = GameObject.FindGameObjectWithTag("UpdateIP").GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        Debug.Log("Established connection with ros");
        reciever = texture.GetComponent<PhotonView>();
        ca_id = rosSocket.Subscribe<geo_msgs.PoseStamped>("/dope/pose_sugar", Sub_dope_pose);
    }
    

    private void Sub_dope_pose(geo_msgs.PoseStamped message)
    {
        //Debug.Log("is called");
        isMessageReceived = true;
        isStart = true;
        /*position_ros.x = (float)message.pose.position.x;
        position_ros.y = (float)message.pose.position.y;
        position_ros.z = (float)message.pose.position.z;

        //Debug.Log("position_ros " + position_ros);
        position_unity = position_ros.Ros2Unity();
        //Debug.Log("position_unity " + position_unity);

        rotation_ros.x = (float)message.pose.orientation.x;
        rotation_ros.y = (float)message.pose.orientation.y;
        rotation_ros.z = (float)message.pose.orientation.z;
        rotation_ros.w = (float)message.pose.orientation.w;
        rotation_unity = rotation_ros.Ros2Unity();*/

    }

    // Update is called once per frame
    void Update()
    {
        timerFirst += Time.deltaTime; //timer
        //Debug.Log(isMessageReceived);
        if (isMessageReceived)
        {
            if (!boolTrue) { reciever.RPC("getReceivedNetwork", RpcTarget.All, true); boolTrue = true; }
            //get the time between each message received
            //Debug.Log("Being called between " + (timerFirst) + "s");
            timerFirst = 0.0f;
            boolFalse = false;

            /*parent.transform.localPosition = position_unity;
            parent.transform.localRotation = rotation_unity;*/

            texture.GetComponent<Renderer>().material = realMaterial;
            //texture.GetComponent<Rigidbody>().isKinematic = true;
            
            isMessageReceived = false;
        }

        else if (isMessageReceived == false && timerFirst > 2f) //if after 2 seconds, no message received => transparent
        {
            texture.GetComponent<Renderer>().material = transparentMaterial;
            if (!boolFalse) { reciever.RPC("getReceivedNetwork", RpcTarget.All, isMessageReceived); boolFalse = true; }
            boolTrue = false;
            //texture.GetComponent<Rigidbody>().isKinematic = false;
        }

        
    }
}
