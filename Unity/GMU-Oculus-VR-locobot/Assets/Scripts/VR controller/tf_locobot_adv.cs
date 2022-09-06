using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using static RosSharp.TransformExtensions; // for transformation (Ros2Unity)
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using sensor_msgs = RosSharp.RosBridgeClient.MessageTypes.Sensor;
using geo_msgs = RosSharp.RosBridgeClient.MessageTypes.Geometry;
using Photon.Pun;
using UnityEngine;

public class tf_locobot_adv : MonoBehaviour
{
    // for one object
    public GameObject parent;
    private Vector3 position_unity;
    private Vector3 position_ros;
    private Quaternion rotation_ros;
    private Quaternion rotation_unity;
    private string ca_id;

    // for multi object
    public GameObject[] dope_object;
    public string[] topic_name;
    private string[] id_name;
    private Vector3[] position_unity_array;
    private Vector3[] position_ros_array;
    private Quaternion[] rotation_ros_array;
    private Quaternion[] rotation_unity_array;

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
        //ca_id = rosSocket.Subscribe<geo_msgs.PoseStamped>("/dope/pose_sugar", Sub_dope_pose);
        ca_id = rosSocket.Subscribe<geo_msgs.PoseStamped>("/dope/pose_sugar", Sub_dope_pose);

        for (int i = 0; i < topic_name.Length ; i++)
        {
            id_name[i] = rosSocket.Subscribe<geo_msgs.PoseStamped>(topic_name[i], Sub_dope_pose_array);
        }
    }

    private void Sub_dope_pose_array(geo_msgs.PoseStamped message)
    {
        //Debug.Log("is called");
        isMessageReceived = true;
        for (int i=0; i< id_name.Length; i++)
        {
            position_ros_array[i].x = (float)message.pose.position.x;
            position_ros_array[i].y = (float)message.pose.position.y;
            position_ros_array[i].z = (float)message.pose.position.z;

            //Debug.Log("position_ros " + position_ros);
            position_unity_array[i] = position_ros_array[i].Ros2Unity();
            //Debug.Log("position_unity " + position_unity);

            rotation_ros_array[i].x = (float)message.pose.orientation.x;
            rotation_ros_array[i].y = (float)message.pose.orientation.y;
            rotation_ros_array[i].z = (float)message.pose.orientation.z;
            rotation_ros_array[i].w = (float)message.pose.orientation.w;
            rotation_unity_array[i] = rotation_ros_array[i].Ros2Unity();
        }

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
            for (int i = 0; i < id_name.Length; i++)
            {
                dope_object[i].transform.localPosition = position_unity;
                dope_object[i].transform.localRotation = rotation_unity;
            }
            parent.transform.localPosition = position_unity;
            parent.transform.localRotation = rotation_unity;
  
        }




    }
}
