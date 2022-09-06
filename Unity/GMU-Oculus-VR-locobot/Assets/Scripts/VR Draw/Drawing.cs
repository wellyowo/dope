using System.Collections;
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
    private List<GameObject> lines = new List<GameObject>(); //lists of lines
    private LineRenderer newLine;
    public float startWidth = 0.05f, endWidth = 0.05f;
    private int numClicks = 0;
    
    private float gripValue, gripLeftValue, rightTriggerValue;

    void Start()
    {
    }

    void Update()
    {

        gripValue = controllerInput.GetComponent<ControllersManager>().getRightGrip();
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
        }

        else
        {
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
        for (int i = 0; i < lines.Count; i++)
        {

            if (lines[i] != null) { Destroy(lines[i]); }
        }
        lines = new List<GameObject>();
    }
}
