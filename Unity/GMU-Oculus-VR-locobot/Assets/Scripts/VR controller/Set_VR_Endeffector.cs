using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using Photon.Pun;

public class Set_VR_Endeffector : MonoBehaviour
{
    //private PhotonView photonView;
    private Transform vr_controller;
    // Start is called before the first frame update
    void Start()
    {
       //photonView = GetComponent<PhotonView>();
        XRRig rig = FindObjectOfType<XRRig>();
        vr_controller = rig.transform.Find("Camera Offset/RightHand Controller");
    }

    // Update is called once per frame
    void Update()
    {
        this.transform.position = vr_controller.position;
        this.transform.rotation = vr_controller.rotation;
    }
}
