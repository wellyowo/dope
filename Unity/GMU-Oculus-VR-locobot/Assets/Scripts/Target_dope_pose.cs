using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using static RosSharp.TransformExtensions; // for transformation (Ros2Unity)
using geo_msgs = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Photon.Pun;
using UnityEngine;

public class Target_dope_pose : MonoBehaviour
{
    RosSocket rosSocket;
    private string RosBridgeServerUrl; //IP address

    public string topic_name;
    public GameObject parent;
    public Material transparentMaterial;
    private Material realMaterial;
    public GameObject texture;

    private string ca_id;
    private bool isMessageReceived = false, boolTrue, boolFalse;
    private PhotonView reciever;

    private float timerFirst = 0.0f;

    void Start()
    {

        RosBridgeServerUrl = GameObject.FindGameObjectWithTag("UpdateIP").GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        Debug.Log("Established connection with ros");
        reciever = texture.GetComponent<PhotonView>();
        //ca_id = rosSocket.Subscribe<geo_msgs.PoseStamped>("/dope/pose_sugar", Sub_dope_pose);
        ca_id = rosSocket.Subscribe<geo_msgs.PoseStamped>(topic_name, Sub_dope_pose);

    }
    

    private void Sub_dope_pose(geo_msgs.PoseStamped message)
    {
        //Debug.Log("is called");
        isMessageReceived = true;
    }

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
