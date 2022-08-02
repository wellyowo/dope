using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Pun;

public class NetworkPlayerSpawner : MonoBehaviourPunCallbacks
{
    private GameObject spawnPlayerprefab, TFbot;
    
    private bool firstPlayer;

    public override void OnJoinedRoom()
    {
        base.OnJoinedRoom();
        Debug.Log(PhotonNetwork.CurrentRoom.PlayerCount);
        if (PhotonNetwork.CurrentRoom.PlayerCount.CompareTo(1) > 0 )
        {
            firstPlayer = true;
        }

        if (!firstPlayer)
        {
            
            spawnPlayerprefab = PhotonNetwork.Instantiate("Fake locobot", transform.position, transform.rotation);
        }
        else
        {
            //TFbot = PhotonNetwork.Instantiate("Locobot_tf", transform.position, transform.rotation);
            PhotonNetwork.Instantiate("Camera Dope View", new Vector3(0.139f, 0.601f, 3.22f), transform.rotation * Quaternion.Euler(90, 90, -90));
            PhotonNetwork.Instantiate("Real locobot", transform.position, transform.rotation);
        }
    }
    
    public override void OnLeftRoom()
    {
        base.OnLeftRoom();
        if (firstPlayer)
        {
            PhotonNetwork.Destroy(spawnPlayerprefab);
        }
        
    }
}