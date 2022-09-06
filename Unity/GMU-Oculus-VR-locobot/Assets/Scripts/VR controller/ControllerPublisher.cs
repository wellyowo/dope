using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using std_msgs = RosSharp.RosBridgeClient.MessageTypes.Std;
using UnityEngine;
using UnityEngine.XR;

public class ControllerPublisher : MonoBehaviour
    {
        RosSocket rosSocket;
        string primary_button;
        string scondary_button;

        //VR Device
        public InputDeviceCharacteristics controllerCharacteristics;

        public string FrameId = "Unity";
        public GameObject IP;
        public GameObject Controller;

        private string RosBridgeServerUrl; 
        private bool primaryButtonValue, secondaryButtonValue;

    void Start()
    {

        //RosSocket
        RosBridgeServerUrl = IP.GetComponent<UpdateRosIP>().getRosIP();
        rosSocket = new RosSocket(new RosSharp.RosBridgeClient.Protocols.WebSocketNetProtocol(RosBridgeServerUrl));
        Debug.Log("Established connection with ros");

        //Topic name
        primary_button = rosSocket.Advertise<std_msgs.Bool>("/vr/right/primarybutton");
        scondary_button = rosSocket.Advertise<std_msgs.Bool>("/vr/right/secondarybutton");

    }


    void Update()
     {

        // primaryButton
        primaryButtonValue = Controller.GetComponent<ControllersManager>().getPrimaryButton();
        if (primaryButtonValue == true)
        {
            std_msgs.Bool message_p = new std_msgs.Bool
            {
                data = primaryButtonValue
            };

            rosSocket.Publish(primary_button, message_p);
        }

        // secondaryButton
        secondaryButtonValue = Controller.GetComponent<ControllersManager>().getSecondaryButton();
        if (secondaryButtonValue)
        {
            std_msgs.Bool message_s = new std_msgs.Bool
            {
                data = secondaryButtonValue
            };
            rosSocket.Publish(scondary_button, message_s);
        }

    }
}
    


