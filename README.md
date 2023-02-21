# WFH_locobot
A public version for WFH-VR teleoperation system (Robot : LoCoBot)

This system is used in this paper 

[WFH-VR: Teleoperating a Robot Arm to set a Dining Table across the Globe via Virtual Reality](https://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=9981729)

```
@INPROCEEDINGS{9981729,
  author={Yim, Lai Sum and Vo, Quang TN and Huang, Ching-I and Wang, Chi-Ruei and McQueary, Wren and Wang, Hsueh-Cheng and Huang, Haikun and Yu, Lap-Fai},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={WFH-VR: Teleoperating a Robot Arm to set a Dining Table across the Globe via Virtual Reality}, 
  year={2022},
  volume={},
  number={},
  pages={4927-4934},
  doi={10.1109/IROS47612.2022.9981729}}
```

# Basic requirement 
Robot : LoCoBot

VR device : Oculus quest 2

Unity version : 2020.3.36

Unity package : 
- XR interaction Toolkit (version : 2.0.2)
- RosSharp 
- PUN2
- Animation Rigging
- Final-ik 

# Basic setup

##  Setup in unity
### ROS sharp
1. Clone from this repo : https://github.com/yimlaisum2014/RosSharp
2. Copy RosSharp folder into your_unity_project/Assets

### VR devices
1. Oculus XR Plugin
Goto Edit > Project Settings > XR Plug-in Management, set the oculus 
2. XR interaction Toolkit [Link]
3. Import starter Assets

### PUN2
1. Download and import Photon Unity Networking packages
2. Get the Appid from your PUN2 account
3. Window >  Photon Unity Networking > Highlight server setting (Make sure the App Id PUN and Dev Region are been set)

P.S Having same name of websocket-sharp.dll with RosSharp and PUN2 package, please delete one of them.



# Usage 
If you are interested build up WFH-VR system from scratch.

You could follow the following tutorial:

1. 

If you are interested in develop your own feature based on WFH-VR system or use WFH-VR system

You could follow the following instrunction: 

1. You should download this unity package. [Link](https://drive.google.com/file/d/1kydMeaIZmJhMl7KHf5UtlN6aVfZTzvmk/view?usp=share_link)
2. Create a New Unity Project and following the Basic setup section
3. Import WFH-VR unity package
4. Choose the scense fit your network setup and the feature
    - [[Turial Link](Tutorial\P_Local_w_VS.md)] Local mode : wired ethernet connection with VR-VS mode 
    - [[Turial Link](Tutorial\P_Local_w_3DO.md)] Local mode : wired ethernet connection with VR-3DO mode (pose-estimation by DOPE) 
    - [[Turial Link](Tutorial\P_Global_w_VS.md)] Glocal mode : PUN2 cloud framework with VR-VS mode 
    - [[Turial Link](Tutorial\P_Global_w_3DO.md)] Glocal mode : PUN2 cloud framework with VR-3DO mode (pose-estimation by DOPE) 

