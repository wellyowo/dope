using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.IO;
using Photon.Pun;

public class FeedbackManager : MonoBehaviour
{
    private bool trigger_gripper, joint_trigger, image_trigger;
    private bool trigger_pri, trigger_sec, trigger_joint;
    private PhotonView photonView;
    private string filePath;
    const string format = "yyyy'-'MM'-'dd' 'HH':'mm':'ss'.'ffffff''";
    private byte[] imageData;
    private Texture2D texture2D;
    public MeshRenderer meshRenderer;

    // Start is called before the first frame update
    void Start()
    {
        photonView = GetComponent<PhotonView>();

        //save file stat
        filePath = Application.dataPath + "/data.csv";
        FileStream fs = new FileStream(filePath, FileMode.Create, FileAccess.Write);
        System.IO.FileInfo FileAttribute = new FileInfo(filePath);
        FileAttribute.Attributes = FileAttributes.Normal;
        StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.Default);
        sw.Write("opertor_side - tigger_name" + "," + "time" + "\n");
        sw.Close();
        //end file initial
    }

    private void savetofile(String tigger_name, String time)
    {

        //Debug.Log("Save");

        FileStream fs = new FileStream(filePath, FileMode.Append, FileAccess.Write);
        StreamWriter sw = new StreamWriter(fs, System.Text.Encoding.Default);

        sw.Write(tigger_name + "," + time + "\n");
        sw.Close();
    }
    void Update()
    {

    }

    // detect
    [PunRPC]
    public void getrosgt_Network(bool trigger_gripper) // When robot side update the gripper state
    {
        this.trigger_gripper = trigger_gripper;
        if(this.trigger_gripper == true)
        {
            savetofile("opertor_side - gripper_trigger_from_ros", DateTime.UtcNow.ToString(format));
        }
        Debug.Log("2 - save gripper trigger data");
    }

    [PunRPC]
    public void getrosjt_Network(bool joint_trigger)
    {
        this.joint_trigger = joint_trigger;
        if (this.joint_trigger == true)
        {
            savetofile("opertor_side - joint_trigger_from_ros", DateTime.UtcNow.ToString(format));
            Debug.Log("3 - save joint trigger data");
        }
    }

    // pri/sec/joint
    [PunRPC]
    public void getrospri_Network(bool pri_fb) // When robot side update the gripper state
    {
        this.trigger_joint = pri_fb;
        if (this.trigger_joint == true)
        {
            savetofile("opertor_side - trigger_joint_from_ros", DateTime.UtcNow.ToString(format));
        }
        Debug.Log("2 - save trigger_joint data");
    }

    [PunRPC]
    public void getrossec_Network(bool sec_fb) // When robot side update the gripper state
    {
        this.trigger_sec = sec_fb;
        if (this.trigger_sec == true)
        {
            savetofile("opertor_side - trigger_sec_from_ros", DateTime.UtcNow.ToString(format));
        }
        Debug.Log("2 - save trigger_sec data");
    }

    [PunRPC]
    public void getrosjoint_Network(bool joint_fb) // When robot side update the gripper state
    {
        this.trigger_pri = joint_fb;
        if (this.trigger_pri == true)
        {
            savetofile("opertor_side - trigger_pri_from_ros", DateTime.UtcNow.ToString(format));
        }
        Debug.Log("2 - save trigger_pri data");
    }

    //[PunRPC]
    //public void getrosim_Network(byte[] imageData)
    //{
    //    this.imageData = imageData;
    //    texture2D.LoadImage(this.imageData);
    //    texture2D.Apply();
    //    meshRenderer.material.SetTexture("_MainTex", texture2D);
    //}

}
