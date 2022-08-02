/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using Photon.Pun;
namespace RosSharp.RosBridgeClient
{
    [RequireComponent(typeof(RosConnector))]
    public class ImageSubscriber : UnitySubscriber<MessageTypes.Sensor.CompressedImage>
    {
        public MeshRenderer meshRenderer;

        private Texture2D texture2D;
        private PhotonView photonView;
        private byte[] imageData;
        private bool isMessageReceived;

        private float timeWait;

        protected override void Start()
        {
			base.Start();
            texture2D = new Texture2D(1, 1);
            photonView = GetComponent<PhotonView>();
            meshRenderer.material = new Material(Shader.Find("Standard"));
        }
        private void Update()
        {
            timeWait += Time.deltaTime;
            //photonView.RPC("getReceived", RpcTarget.All, isMessageReceived);
            if (isMessageReceived)
            {
                ProcessMessage();
                if (timeWait > 1.5f && GetComponent<CameraSync>() != null)
                {
                    //Debug.Log("ok");
                    photonView.RPC("getImage", RpcTarget.All, imageData);
                    timeWait = 0f;
                }
            }

        }

        protected override void ReceiveMessage(MessageTypes.Sensor.CompressedImage compressedImage)
        {
            imageData = compressedImage.data;
            isMessageReceived = true;
        }

        private void ProcessMessage()
        {
            texture2D.LoadImage(imageData);
            texture2D.Apply();

            meshRenderer.material.SetTexture("_MainTex", texture2D);
            isMessageReceived = false;
        }

    }
}

