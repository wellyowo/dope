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

    public GameObject parent;
    /*public Material transparentMaterial;
    public Material realMaterial;*/
    //public GameObject texture;

    private Vector3 position_unity;
    private Vector3 position_ros;
    private Quaternion rotation_ros;
    private Quaternion rotation_unity;
    private string ca_id;



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

    //locobot_gripper
    public GameObject locobot_gripper;
    private string ag_pos_id;
    private string ag_rot_id;
    private Vector3 ag_position_unity;
    private Vector3 ag_position_ros;
    private Quaternion ag_rotation_unity_quat;
    private Quaternion ag_rotation_ros;

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

        //string ca_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_ca_position", SubscriptionHandler);
        oa_pos_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_oa_position", sub_oa_position); //obom to arm_base_link position
        oa_rot_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_oa_rotation", sub_oa_rotation); //obom to arm_base_link rotation
        ac_pos_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_ac_position", sub_ac_position); //arm_base_link to camera_color_optical_frame postion
        ac_rot_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_ac_rotation", sub_ac_rotation); //arm_base_link to camera_color_optical_frame rotation
        //ag_pos_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_ag_position", sub_ag_position); //arm_base_link to gripper_link postion
        //ag_rot_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_ag_rotation", sub_ag_rotation); //arm_base_link to gripper_link rotation
        acs_pos_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_acs_position", sub_acs_position); //arm_base_link to ex_camera_side_link postion
        acs_rot_id = rosSocket.Subscribe<std_msgs.Float32MultiArray>("/tf_acs_rotation", sub_acs_rotation); //arm_base_link to ex_camera_side_link rotation

        ca_id = rosSocket.Subscribe<geo_msgs.PoseStamped>("/dope/pose_sugar", Sub_dope_pose);
    }

    private void Sub_dope_pose(geo_msgs.PoseStamped message)
    {
        //Debug.Log("is called");
        isMessageReceived = true;

        position_ros.x = (float)message.pose.position.x;
        position_ros.y = (float)message.pose.position.y;
        position_ros.z = (float)message.pose.position.z;

        //Debug.Log("position_ros " + position_ros);
        position_unity = position_ros.Ros2Unity();
        //Debug.Log("position_unity " + position_unity);

        rotation_ros.x = (float)message.pose.orientation.x;
        rotation_ros.y = (float)message.pose.orientation.y;
        rotation_ros.z = (float)message.pose.orientation.z;
        rotation_ros.w = (float)message.pose.orientation.w;
        rotation_unity = rotation_ros.Ros2Unity();

    }

    //obom to arm_base_link
    private void sub_oa_position(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        oa_position_ros.x = (float)message.data[0];
        oa_position_ros.y = (float)message.data[1];
        oa_position_ros.z = (float)message.data[2];
        //ROS2Unity (-vector3.y, vector3.z, vector3.x)
        oa_position_unity = oa_position_ros.Ros2Unity();

    }

    private void sub_oa_rotation(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        oa_rotation_ros.x = (float)message.data[0];
        oa_rotation_ros.y = (float)message.data[1];
        oa_rotation_ros.z = (float)message.data[2];
        oa_rotation_ros.w = (float)message.data[3];
        //ROS2Unity (quaternion.y, -quaternion.z, -quaternion.x, quaternion.w)
        oa_rotation_unity_quat = oa_rotation_ros.Ros2Unity();

    }
    //arm_base_link to camera_color_optical_frame
    private void sub_ac_position(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        ac_position_ros.x = (float)message.data[0];
        ac_position_ros.y = (float)message.data[1];
        ac_position_ros.z = (float)message.data[2];
        //ROS2Unity (-vector3.y, vector3.z, vector3.x)
        ac_position_unity = ac_position_ros.Ros2Unity();
        //Debug.Log("ac_position_unity_x " + ac_position_unity[0]);
        //Debug.Log("ac_position_ros_y " + ac_position_ros[1]);
        //Debug.Log("ac_position_unity_y " + ac_position_unity[1]);
        //Debug.Log("ac_position_ros_z " + ac_position_ros[2]);
        //Debug.Log("ac_position_unity_z " + ac_position_unity[2]);
        //Debug.Log("ac_position_ros_x " + ac_position_ros[0]);
    }

    private void sub_ac_rotation(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        ac_rotation_ros.x = (float)message.data[0];
        ac_rotation_ros.y = (float)message.data[1];
        ac_rotation_ros.z = (float)message.data[2];
        ac_rotation_ros.w = (float)message.data[3];
        //ROS2Unity (quaternion.y, -quaternion.z, -quaternion.x, quaternion.w)
        ac_rotation_unity_quat = ac_rotation_ros.Ros2Unity();
        //Debug.Log("ac_rotation_unity_quat_x " + ac_rotation_unity_quat[0]);
        //Debug.Log("ac_rotation_ros_y " + ac_rotation_ros[1]);
        //Debug.Log("ac_rotation_unity_quat_y " + ac_rotation_unity_quat[1]);
        //Debug.Log("ac_rotation_ros_z " + ac_rotation_ros[2]);
        //Debug.Log("ac_rotation_unity_quat_z " + ac_rotation_unity_quat[2]);
        //Debug.Log("ac_rotation_ros_x " + ac_rotation_ros[0]);
        //Debug.Log("ac_rotation_unity_quat_w " + ac_rotation_unity_quat[3]);
        //Debug.Log("ac_rotation_ros_w " + ac_rotation_ros[3]);

    }

    private void sub_ag_position(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        ag_position_ros.x = (float)message.data[0];
        ag_position_ros.y = (float)message.data[1];
        ag_position_ros.z = (float)message.data[2];
        ag_position_unity = ag_position_ros.Ros2Unity();
        //Debug.Log("ag_position_unity_x " + ag_position_unity[0]);
        //Debug.Log("ag_position_ros_y " + ag_position_ros[1]);

    }

    private void sub_ag_rotation(std_msgs.Float32MultiArray message)
    {
        isMessageReceived = true;

        ag_rotation_ros.x = (float)message.data[0];
        ag_rotation_ros.y = (float)message.data[1];
        ag_rotation_ros.z = (float)message.data[2];
        ag_rotation_ros.w = (float)message.data[3];
        ag_rotation_unity_quat = ag_rotation_ros.Ros2Unity();
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
            locobot_arm.transform.localPosition = oa_position_unity; // 0.0001868025 , 0.1075625 , 0.09729982
            locobot_arm.transform.localRotation = oa_rotation_unity_quat; // 0, 0.11 ,0
            locobot_camera.transform.localPosition = ac_position_unity; // -0.0283302 , 0.4797385 , -0.05248966
            locobot_camera.transform.localRotation = ac_rotation_unity_quat; // 0.642 , 92.353 , 92.177
            //locobot_gripper.transform.localPosition = ag_position_unity; // gripper posistion
            //locobot_gripper.transform.localRotation = ag_rotation_unity_quat; // gripper rotation

            parent.transform.localPosition = position_unity;
            parent.transform.localRotation = rotation_unity;
            locobot_camera_side.transform.localPosition = acs_position_unity;
            locobot_camera_side.transform.localRotation = acs_rotation_unity_quat;
        }




    }
}
