using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using static RosSharp.TransformExtensions; // for transformation (Ros2Unity)
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using geo_msgs = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Photon.Pun;
using UnityEngine;

public class tf_locobot : MonoBehaviour
{
    // for one object
    public GameObject parent,Bar, Cup;
    private Vector3 obj1_position_unity, obj2_position_unity, obj3_position_unity;
    private Vector3 position_ros;
    private Quaternion rotation_ros;
    private Quaternion obj1_rotation_unity, obj2_rotation_unity, obj3_rotation_unity;
    private string ca_id, cup_id, bar_id;

    RosSocket rosSocket;
    private string RosBridgeServerUrl; //IP address
    //locobot_arm
    public GameObject locobot_arm;
    private Vector3 oa_position_unity;
    private Vector3 oa_position_ros;
    private Quaternion oa_rotation_unity_quat;
    private Quaternion oa_rotation_ros;
    private string oa_pos_id;
    private string oa_rot_id;

    //locobot_camera
    public GameObject locobot_camera;
    private string ac_pos_id;
    private string ac_rot_id;
    private Vector3 ac_position_unity;
    private Vector3 ac_position_ros;
    private Quaternion ac_rotation_unity_quat;
    private Quaternion ac_rotation_ros;


    //locobot_camera_side
    public GameObject locobot_camera_side;
    private string acs_pos_id;
    private string acs_rot_id;
    private Vector3 acs_position_unity;
    private Vector3 acs_position_ros;
    private Quaternion acs_rotation_unity_quat;
    private Quaternion acs_rotation_ros;

    private PhotonView photonView;
    private bool isMessageReceived;

    // Start is called before the first frame update
    void Start()
    {
        photonView = GetComponent<PhotonView>();

        RosBridgeServerUrl = GameObject.FindGameObjectWithTag("UpdateIP").GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        Debug.Log("Established connection with ros");

        oa_pos_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_oa_position", sub_oa_position); //obom to arm_base_link position
        oa_rot_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_oa_rotation", sub_oa_rotation); //obom to arm_base_link rotation
        ac_pos_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_ac_position", sub_ac_position); //arm_base_link to camera_color_optical_frame postion
        ac_rot_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_ac_rotation", sub_ac_rotation); //arm_base_link to camera_color_optical_frame rotation
        acs_pos_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_acs_position", sub_acs_position); //arm_base_link to ex_camera_side_link postion
        acs_rot_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_acs_rotation", sub_acs_rotation); //arm_base_link to ex_camera_side_link rotation
        
        ca_id = rosSocket.Subscribe<geo_msgs.PoseStamped>("/dope/pose_sugar", Sub_dope_pose);
        cup_id = rosSocket.Subscribe<geo_msgs.PoseStamped>("/dope/pose_Water_Cup", Sub_cup_pose);
        bar_id = rosSocket.Subscribe<geo_msgs.PoseStamped>("/dope/pose_GranolaBars", Sub_bar_pose);

    }

    private static Vector3 R2U_Postion(geo_msgs.Point tran_pose)
    {
        Vector3 position_unity;
        Vector3 position_ros;
        position_ros.x = (float)tran_pose.x - (float)0.085; //0.125 object y 2/23
        position_ros.y = (float)tran_pose.y + (float)0.025; //0.025
        position_ros.z = (float)tran_pose.z - (float)0.045; //0.045 
        position_unity = position_ros.Ros2Unity();
        return position_unity;
    }

    private static Quaternion R2U_Rotation(geo_msgs.Quaternion tran_rotation)
    {
        Quaternion rotation_ros;
        Quaternion rotation_unity;
        rotation_ros.x = (float)tran_rotation.x;
        rotation_ros.y = (float)tran_rotation.y;
        rotation_ros.z = (float)tran_rotation.z;
        rotation_ros.w = (float)tran_rotation.w;
        rotation_unity = rotation_ros.Ros2Unity();
        return rotation_unity;
    }


    private void Sub_dope_pose(geo_msgs.PoseStamped message)
    {
        //Debug.Log("is called");
        isMessageReceived = true;

        Vector3 dope_unity_position = R2U_Postion(message.pose.position);
        obj1_position_unity = dope_unity_position;
        Quaternion dope_unity_rotation = R2U_Rotation(message.pose.orientation);
        obj1_rotation_unity = dope_unity_rotation;
    }
    private void Sub_cup_pose(geo_msgs.PoseStamped message)
    {
        //Debug.Log("is called");
        isMessageReceived = true;

        Vector3 dope_unity_position = R2U_Postion(message.pose.position);
        obj2_position_unity = dope_unity_position;
        Quaternion dope_unity_rotation = R2U_Rotation(message.pose.orientation);
        obj2_rotation_unity = dope_unity_rotation;
    }

    private void Sub_bar_pose(geo_msgs.PoseStamped message)
    {
        //Debug.Log("is called");
        isMessageReceived = true;

        Vector3 dope_unity_position = R2U_Postion(message.pose.position);
        obj3_position_unity = dope_unity_position;
        Quaternion dope_unity_rotation = R2U_Rotation(message.pose.orientation);
        obj3_rotation_unity = dope_unity_rotation;
    }


    //obom to arm_base_link
    private void sub_oa_position(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        oa_position_ros.x = (float)message.data[0];
        oa_position_ros.y = (float)message.data[1];
        oa_position_ros.z = (float)message.data[2];
        oa_position_unity = oa_position_ros.Ros2Unity();

    }

    private void sub_oa_rotation(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        oa_rotation_ros.x = (float)message.data[0];
        oa_rotation_ros.y = (float)message.data[1];
        oa_rotation_ros.z = (float)message.data[2];
        oa_rotation_ros.w = (float)message.data[3];
        oa_rotation_unity_quat = oa_rotation_ros.Ros2Unity();

    }
    //arm_base_link to camera_color_optical_frame
    private void sub_ac_position(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        ac_position_ros.x = (float)message.data[0];
        ac_position_ros.y = (float)message.data[1];
        ac_position_ros.z = (float)message.data[2];
        ac_position_unity = ac_position_ros.Ros2Unity();

    }

    private void sub_ac_rotation(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        ac_rotation_ros.x = (float)message.data[0];
        ac_rotation_ros.y = (float)message.data[1];
        ac_rotation_ros.z = (float)message.data[2];
        ac_rotation_ros.w = (float)message.data[3];
        ac_rotation_unity_quat = ac_rotation_ros.Ros2Unity();


    }

    //arm_base_link to camera_side_link
    private void sub_acs_position(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        acs_position_ros.x = (float)message.data[0];
        acs_position_ros.y = (float)message.data[1];
        acs_position_ros.z = (float)message.data[2];
        acs_position_unity = acs_position_ros.Ros2Unity();
    }
    private void sub_acs_rotation(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        acs_rotation_ros.x = (float)message.data[0];
        acs_rotation_ros.y = (float)message.data[1];
        acs_rotation_ros.z = (float)message.data[2];
        acs_rotation_ros.w = (float)message.data[3];
        acs_rotation_unity_quat = acs_rotation_ros.Ros2Unity();
    }

    // Update is called once per frame
    void Update()
    {
        if(isMessageReceived && photonView.IsMine)
        {
            // Fix_tf
            locobot_arm.transform.localPosition = oa_position_unity; // 0.0001868025 , 0.1075625 , 0.09729982
            locobot_arm.transform.localRotation = oa_rotation_unity_quat; // 0, 0.11 ,0
            locobot_camera.transform.localPosition = ac_position_unity; // -0.0283302 , 0.4797385 , -0.05248966
            locobot_camera.transform.localRotation = ac_rotation_unity_quat; // 0.642 , 92.353 , 92.177
            locobot_camera_side.transform.localPosition = acs_position_unity;
            locobot_camera_side.transform.localRotation = acs_rotation_unity_quat;

            // Dope position
            parent.transform.localPosition = obj1_position_unity;
            parent.transform.localRotation = obj1_rotation_unity;
            Bar.transform.localPosition = obj3_position_unity;
            Bar.transform.localRotation = obj3_rotation_unity;
            Cup.transform.localPosition = obj2_position_unity;
            Cup.transform.localRotation = obj2_rotation_unity;



        }




    }
}
