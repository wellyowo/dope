﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.XR;
using RosSharp.RosBridgeClient;
using System;
using Photon.Pun;

public class Drawing : MonoBehaviour
{
    public Transform rightController;
    public Material lineMaterial;
    public float destroyTimer = 4f;
    public GameObject controllerInput;
    public GameObject Connector;
    public DateTime gra_trigger_time; //triger feedback - time
    public bool gra_trigger = false; //triger feedback - bool
    private List<GameObject> lines = new List<GameObject>(); //lists of lines
    private LineRenderer newLine;
    public float startWidth = 0.05f, endWidth = 0.05f;
    private int numClicks = 0;
    
    private float gripValue, gripLeftValue, rightTriggerValue;

    // Start is called before the first frame update
    void Start()
    {
        //Connector = GameObject.Find("Robot Connector (Fake)");
    }

    // Update is called once per frame
    void Update()
    {
        /*if(Connector == null)
        {
            Connector = GameObject.Find("Fake locobot(Clone)");
        }*/
        gripValue = controllerInput.GetComponent<ControllersManager>().getRightGrip();
        gripLeftValue = controllerInput.GetComponent<ControllersManager>().getLeftGrip();
        rightTriggerValue = controllerInput.GetComponent<ControllersManager>().getRightTrigger();
        
        if (gripLeftValue < 0.1f)
        {
            trajectoryMode(gripValue);
        }
        
        else
        {
            realTimemode();
        }
    }

    private void trajectoryMode(float gripValue)
    {

        if (gripValue > 0.7f)
        {
            newLine.enabled = true;
            newLine.positionCount = numClicks + 1;
            newLine.SetPosition(numClicks, rightController.position);
            numClicks++;
            Debug.Log("Gip Pressed.");
            Connector.GetComponent<JointStatePublisher>().enabled = true;
            gra_trigger_time = DateTime.UtcNow; //triger feedback - time
            gra_trigger = true; //triger feedback - bool
        }

        else
        {
            gra_trigger = false;
            if (gripValue > 0.1f)
            {
                GameObject draw = new GameObject();
                newLine = draw.AddComponent<LineRenderer>();

                newLine.startWidth = startWidth;
                newLine.endWidth = endWidth;
                newLine.material = lineMaterial;
                newLine.enabled = false;

                lines.Add(newLine.gameObject);
                numClicks = 0;
            }

            else if (gripValue < 0.1)
            {
                delLines();
            }
            Connector.GetComponent<JointStatePublisher>().enabled = false;
        }
    }

    private void realTimemode()
    {
        Connector.GetComponent<JointStatePublisher>().enabled = true;
    }

    private void delLines()
    {
        //Debug.Log("called");
        for (int i = 0; i < lines.Count; i++)
        {

            if (lines[i] != null) { Destroy(lines[i]); }
        }
        lines = new List<GameObject>();
    }
}