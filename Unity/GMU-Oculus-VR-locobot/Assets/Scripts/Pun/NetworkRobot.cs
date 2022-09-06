using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Pun;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using RosSharp.RosBridgeClient;


public class NetworkRobot : MonoBehaviour
{
    private PhotonView photonView;
    public GameObject targetArm;
    public GameObject targetWrist;
    public GameObject targetRotation;

    private JointStatePublisher publisher;
    private PhotonView controllerPV;
    private Transform vr_controller;
    private InputDevice rightController, leftController;
    private float xVR;
    private float zObject, zVR;
    private List<InputDevice> devices = new List<InputDevice>();

    void Start()
    {
        controllerPV = GameObject.FindWithTag("Controllers").GetComponent<PhotonView>();
        photonView = GetComponent<PhotonView>();

        //publisher = GetComponent<JointStatePublisher>();

        XRRig rig = FindObjectOfType<XRRig>();
        vr_controller = rig.transform.Find("Camera Offset/RightHand Controller");

        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, devices);
        if (devices.Count > 0)
        {
            rightController = devices[0];
        }

        InputDevices.GetDevicesAtXRNode(XRNode.LeftHand, devices);
        if (devices.Count > 0)
        {
            leftController = devices[0];
        }
    }

    void Update()
    {
        zVR = vr_controller.eulerAngles.z;
        xVR = vr_controller.eulerAngles.x;
        rightController.TryGetFeatureValue(CommonUsages.grip, out float gripRightValue);
        rightController.TryGetFeatureValue(CommonUsages.primaryButton, out bool primaryButtonValue);
        rightController.TryGetFeatureValue(CommonUsages.secondaryButton, out bool secondaryButtonValue);
        leftController.TryGetFeatureValue(CommonUsages.grip, out float gripLeftValue);

        if (photonView.IsMine)
        {
            targetArm.transform.position = vr_controller.position;
            targetArm.transform.rotation = vr_controller.rotation;

            targetWrist.transform.eulerAngles = new Vector3(xVR, transform.eulerAngles.y, transform.eulerAngles.z);
            targetRotation.transform.eulerAngles = new Vector3(transform.eulerAngles.x, transform.eulerAngles.y, zVR);

            controllerPV.RPC("getNetworkRightGrip", RpcTarget.All, gripRightValue);
            controllerPV.RPC("getNetworkLeftgrip", RpcTarget.All, gripLeftValue);
            controllerPV.RPC("getNetworkRightPrim", RpcTarget.All, primaryButtonValue);
            controllerPV.RPC("getNetworkRightSec", RpcTarget.All, secondaryButtonValue);
        }
        
    }

}
