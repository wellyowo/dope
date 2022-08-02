using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;

public class Set_Wrist_rotation : MonoBehaviour
{
    private Transform vr_controller;

    private float xVR;
    // Start is called before the first frame update
    void Start()
    {
        XRRig rig = FindObjectOfType<XRRig>();
        vr_controller = rig.transform.Find("Camera Offset/RightHand Controller");
    }

    // Update is called once per frame
    void Update()
    {
        xVR = vr_controller.eulerAngles.x;
        this.transform.eulerAngles = new Vector3(xVR, transform.eulerAngles.y, transform.eulerAngles.z);
    }
}