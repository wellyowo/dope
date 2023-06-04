# dope

## clone repo
```
    $ git clone --recursive git@github.com:wellyowo/dope.git
```

*You can run this code on a GPU computer and CPU computer, but if you want to execute DOPE program, please change GPU computer to run*

---
|Device         |GPU            |IP                         |
|:-------------:|:-------------:|:-------------------------:|
|LoCoBot        |No             |10.0.0.2 and 192.168.50.2  |
|VR PC          |Yes            |10.0.0.3                   |
|DOPE PC        |Yes            |192.168.50.3               |

![Teaser](figures/network.PNG)
---

## Building docker image
```
    $ cd Docker && source build.sh
```

## Download weights

Download [the weights](https://drive.google.com/open?id=1DfoA3m_Bm0fW8tOWXGVxi4ETlLEAgmcg) and save them to the `weights` folder, *i.e.*, `~/catkin_ws/src/Deep_Object_Pose/weights/`.

## How to run
On LoCoBot
```
    $ cd Docker && source docker_run.sh cpu
    Docker$ cd catkin_ws && catkin_make
    Docker$ source environment.sh 192.168.50.2 192.168.50.2
    Docker$ roslaunch realsense2_camera rs_rgbd.launch
    Docker$ roslaunch rosbridge_server rosbridge_websocket.launch
```
On DOPE PC
```
    $ cd Docker && source docker_run.sh gpu
    Docker$ cd catkin_ws && catkin_make
    Docker$ source environment.sh 192.168.50.3 192.168.50.3
    Docker$ roslaunch rosbridge_server rosbridge_websocket.launch
    Docker$ rosrun topic_rosbridge image_rosbridge.py
    Docker$ rosrun image_transport republish compressed in:=/camera/color/image_raw raw out:=/dope/image_raw
    Docker$ roslaunch dope dope.launch
```
## For across the globe dope
```
    $ cd Docker && source docker_run.sh gpu
    Docker$ cd catkin_ws && catkin_make #only need at first time
    Docker$ source environment.sh (locobot_ip) (GPU_computer_ip)
    Docker$ roslaunch dope_pose object_left_trans.launch pose:="result of hand_eye_calibration x y z quaternion" 
    e.g. pose:="0.443019 0.515959 0.742330 0.323063 0.307332 -0.627258 0.638534"

```

## If you want to enter same container
```
    $ cd Docker && source docker_join.sh
    Docker$ source environment.sh ros_master_ip ros_ip
