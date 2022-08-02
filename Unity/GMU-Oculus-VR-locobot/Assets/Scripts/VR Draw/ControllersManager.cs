using RosSharp.RosBridgeClient;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using Photon.Pun;

public class ControllersManager : MonoBehaviour
{
    private InputDevice rightController;
    private InputDevice leftController;
    private List<InputDevice> devices = new List<InputDevice>();

    private float gripRightValue, gripLeftValue;
    private bool primaryButtonValue, secondaryButtonValue;
    private bool pri_trigger, sec_trigger, gra_trigger;
    // Start is called before the first frame update
    void Start()
    {
        InputDevices.GetDevicesAtXRNode(XRNode.LeftHand, devices);
        if (devices.Count > 0)
        {
            leftController = devices[0];
        }

        InputDevices.GetDevicesAtXRNode(XRNode.RightHand, devices);
        if (devices.Count > 0)
        {
            rightController = devices[0];
        }
    }


    //-------RIGHT CONTROLLER------------//
    public float getRightGrip()
    {
        //rightController.TryGetFeatureValue(CommonUsages.grip, out float gripRightValue);
        return gripRightValue;
    }

    public float getRightTrigger()
    {
        rightController.TryGetFeatureValue(CommonUsages.trigger, out float triggerValue);
        return triggerValue;
    }

    public bool getPrimaryButton()
    {
        return primaryButtonValue;
    }

    public bool getSecondaryButton()
    {
        return secondaryButtonValue;
    }
    // feedback
    public bool getP_T()
    {
        if (this.primaryButtonValue == true)
        {
            pri_trigger = true;
        }
        else
        {
            pri_trigger = false;
        }
        return pri_trigger;
    }

    public bool getS_T()
    {
        if (this.secondaryButtonValue == true)
        {
            sec_trigger = true;
        }
        else
        {
            sec_trigger = false;
        }
        return sec_trigger;
    }

    public bool getG_T()
    {
        if (this.gripRightValue > 0.7f)
        {
            gra_trigger = true;
        }
        else
        {
            gra_trigger = false;
        }
        return gra_trigger;
    }
    // feedback


    [PunRPC]
    public void getNetworkRightGrip(float gripRightValue)
    {
        this.gripRightValue = gripRightValue;
    }

    [PunRPC]
    public void getNetworkRightPrim(bool primaryButtonValue)
    {
        this.primaryButtonValue = primaryButtonValue;
    }

    [PunRPC]
    public void getNetworkRightSec(bool secondaryButtonValue)
    {
        this.secondaryButtonValue = secondaryButtonValue;
    }
    // feedback
    [PunRPC]
    public void getPT_Network(bool pri_trigger)
    {
        this.pri_trigger = pri_trigger;
    }

    [PunRPC]
    public void getST_Network(bool sec_trigger)
    {
        this.sec_trigger = sec_trigger;
    }

    [PunRPC]
    public void getGT_Network(bool gra_trigger)
    {
        this.gra_trigger = gra_trigger;
    }
    // feedback
    //-------------------------------------------------------//

    //-------LEFT CONTROLLER------------//
    public float getLeftGrip()
    {
        return gripLeftValue;
    }


    [PunRPC]
    public void getNetworkLeftgrip(float gripLeftValue)
    {
        this.gripLeftValue = gripLeftValue;
    }

}
