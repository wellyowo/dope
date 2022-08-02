using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;

public class Set_VR_Rotation : MonoBehaviour
{
    private Transform vr_controller;
    private float zObject, zVR;
    // Start is called before the first frame update
    void Start()
    {
        XRRig rig = FindObjectOfType<XRRig>();
        vr_controller = rig.transform.Find("Camera Offset/RightHand Controller");
    }

    // Update is called once per frame
    void Update()
    {
        zVR = vr_controller.eulerAngles.z;
        this.transform.eulerAngles = new Vector3(transform.eulerAngles.x, transform.eulerAngles.y, zVR);
    }
}
