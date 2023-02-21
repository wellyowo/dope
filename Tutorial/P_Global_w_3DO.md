# Local mode :  PUN2 cloud framework with VR-3DO mode

## Application : 
If you would like to setup internet connection between two site by using PUN and using pose-estimation method to update the target object pose.

## Example setup
Operator site : 
- one PC for running unity (Windows OS)
Remote site : 
- one PC fpr running unity (Windows OS)
- one NUC for robot control (Ubuntu)

## Networl setup: 
1. known PUN App ID and region of the server
- ID : 0fbeb5f2-xxxx-xxxx-xxxx-3c50ed985759
- dev region : jp

2. known both computer’s IP address 
- Remote : NUC (10.42.0.2 - ros master) : robot control
- Remote : PC (10.42.0.3) : running unity

## Usage

### Operator site : Unity setting 

#### Step1. Goto the scene (Global with video-streaming)

#### Step2. Goto Game Manager > UpdatedIP
- Assign which object needed to update the ros_ip
    - Default setting : 
        - Robot Connector (real) Robot Connector (human) + 
        - Camera view_1 
        - Camera view_2 (if you have)
- update the ip address with you ros_master_ip address

### Remote site : ROS setting 
Remind : 

- suggest run the script for vr script and locobot script separately because of different workspace.
- unplug another D435-camera first if you have one because due to the original script will launch a D435-camera (pyrobot)

#### 1. Clone repo

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone git@github.com:ARG-NCTU/WFH_locobot.git
```

####  2. Pull Docker

``` bash
docker pull yimlaisum2014/locobot:NUC-vr-py2
```
#### 3. Enter Docker
```
NUC $ cd WFH_locobot/Docker/NUC/vr
NUC $ source docker_run.sh
```

#### 4. Start locobot

```bash
Docker $ cd WFH_locobot/
Docker $ source set_ip.sh
Docker $ source run_locobot.sh
```

#### 5. Start WFH-VE procman
```bash
Docker $ cd WFH_locobot/
Docker $ source set_wfh_workspace_env.sh
Docker $ source start_vr.sh
```

### Procman
![vr_procman](Figures/vr_procman.png)

What script needed to launch :
- ros_core
- robridge
- vr_arm
- *side_camera (if you have extra D435 camera)



