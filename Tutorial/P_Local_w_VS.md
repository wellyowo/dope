# Local mode : wired ethernet connection with VR-VS mode 

## Application : 
You want to set up the local network, in which your operator site and remote site are under the same domain network, connected by wired ethernet or wifi. And you would like to abserate the remote enviroenment by using image streaming.

## Example setup
Operator site : 
- one PC for running unity (Windows OS)
Remote site : 
- one NUC for robot control (Ubuntu)

## Networl setup: 
1. known both computer’s IP address 

- Operator : PC (10.42.0.3)
- Remote : NUC (10.42.0.2 - ros master) : robot control

2. known both computer’s IP address 

## Usage

### Operator site : Unity setting (PC (10.42.0.3))

#### Step1. Goto the scene (Local with video-streaming)

#### Step2. Goto Game Manager > UpdatedIP
- Assign which object needed to update the ros_ip
    - Default setting : 
        - Robot Connector (real) Robot Connector (human) + 
        - Camera view_1 
        - Camera view_2 (if you have)
- update the ip address with you ros_master_ip address

### Remote site : ROS setting (NUC(10.42.0.2))
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



