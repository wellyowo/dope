using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
using Photon.Pun;


public class UpdatePosDope : MonoBehaviour
{
    //public Transform parent;
    public GameObject targetObj;
    private Transform rightConPos;

    private Transform fixedDopePose;
    private Rigidbody rb;
    private GameObject controllers;
    private bool isGrabbing, collided, setOnce = true, isStart = true, isRecieved;

    //private Transform obj;
    // Start is called before the first frame update
    void Start()
    {
        rb = GetComponent<Rigidbody>();
        XRRig rig = FindObjectOfType<XRRig>();
        rightConPos = rig.transform.Find("Camera Offset/RightHand Controller");
        controllers = GameObject.FindWithTag("Controllers");
    }

    // Update is called once per frame
    void Update()
    {
        getGrab();
       if (isRecieved)
        {
            isStart = false;
            rb.isKinematic = true;
            if (!setOnce)
            {
                this.transform.localPosition = new Vector3(-0.1199f, 0.015f, -0.006f);
                this.transform.localRotation = new Quaternion(-0.5f, 0.5f, -0.5f, 0.5f);
                setOnce = true;
            }
            
        } 

       else if (!isRecieved && !isGrabbing && !isStart)
        {
            
            setOnce = false;
            rb.isKinematic = false;
        }
        
       if (collided && isGrabbing)
        {
            rb.isKinematic = true;
            rb.velocity = Vector3.zero;
            rb.MovePosition(rb.position + (rightConPos.position - rb.worldCenterOfMass));
            setOnce = false;
        }
    }

    private void OnTriggerEnter(Collider other)
    {
        //Debug.Log(other.name);
        if (other.name == "finger_r_0" || other.name == "finger_l_0")
        {
            if (isGrabbing)
            {
                collided = true;
            }
            else
            {
                collided = false;
            }
        }
    }

    private void getGrab()
    {
        if (controllers.GetComponent<ControllersManager>().getPrimaryButton())
        {
            isGrabbing = true;
        }

        if (controllers.GetComponent<ControllersManager>().getSecondaryButton())
        {
            isGrabbing = false;
        }
    }

    [PunRPC]
    public void getReceivedNetwork(bool isRecieved)
    {
        this.isRecieved = isRecieved;
        Debug.Log("calling");
    }
}
