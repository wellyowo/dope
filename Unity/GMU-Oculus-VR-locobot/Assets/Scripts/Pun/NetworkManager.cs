using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Photon.Pun;
using Photon.Realtime;

public class NetworkManager : MonoBehaviourPunCallbacks
{
    // Start is called before the first frame update
    void Start()
    {
        ConnectToServer();
    }

    // Update is called once per frame
    void ConnectToServer()
    {
        PhotonNetwork.ConnectUsingSettings();
        Debug.Log("Pun Connecting...");
    }

    public override void OnConnectedToMaster()
    {
        Debug.Log("Pun Connected");
        base.OnConnectedToMaster();
        RoomOptions roomOp = new RoomOptions();
        roomOp.MaxPlayers = 2;
        roomOp.IsVisible = true;
        roomOp.IsOpen = true;
        PhotonNetwork.JoinOrCreateRoom("Room 1", roomOp, TypedLobby.Default);
    }

    public override void OnJoinedRoom()
    {
        Debug.Log("Joined Room");
        base.OnJoinedRoom();
    }

    public override void OnPlayerEnteredRoom(Player newPlayer)
    {
        Debug.Log("New player joined");
        base.OnPlayerEnteredRoom(newPlayer);
    }
}
