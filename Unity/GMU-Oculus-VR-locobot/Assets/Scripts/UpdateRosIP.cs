using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UpdateRosIP : MonoBehaviour
{
    public string WebSocketIP = "ws://10.0.0.228:9090";

    public string getRosIP()
    {
        return WebSocketIP;
    }
}
