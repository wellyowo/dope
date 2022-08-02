
# 1. Run rosbridge server

ssh locobot@locobotgmu.local

## NUC

cd GMU_NCTU_Unity_ROS/Docker/NUC/vr

source docker_run.sh

## Docker
cd GMU_NCTU_Unity_ROS

source install_tools.sh

source catkin_make.sh

source environment.sh

roslaunch rosbridge_server rosbridge_websocket.launch


# 2. Run Robot's launch file
ssh locobot@locobotgmu.local

## NUC

cd GMU_NCTU_Unity_ROS/Docker/NUC/vr

source docker_join.sh


## Docker (If doesn't work use native locobot)

cd GMU_NCTU_Unity_ROS

export ROS_IP=10.0.0.228

export ROS_MASTER_URI=http://10.0.0.228:11311

roslaunch locobot_control main.launch use_camera:=true use_base:=true use_arm:=true


# 3. Compress Image

cd GMU_NCTU_Unity_ROS

export ROS_IP=10.0.0.228 (Change your IP if needed)

export ROS_MASTER_URI=http://10.0.0.228:11311

rosrun image_transport republish raw in:='camera/color/image_raw' compressed out:='camera/color/image_raw'



# 4. Run Dope on Xavier 

## setup
`git clone https://github.com/ARG-NCTU/dope-tx2.git`
`cd dope-tx2`
`source tx2_docker_run.sh`
`cd dope`
`source environment.sh`
`source tx2_catkin_make.sh`

## terminal 1
`cd dope-tx2`
`source tx2_docker_run.sh`
`cd dope`
Xavier-nx $ `source environment.sh`
Docker $ `export ROS_IP=10.42.0.3` #change to your IP
Docker $ `export ROS_MASTER_URI=http://10.42.0.2:11311` #change to your IP
Docker $ `roslaunch dope dope.launch`



# 5. Publish tf-locobot relationship for unity
ssh locobot@locobotgmu.local

## NUC

cd GMU_NCTU_Unity_ROS/Docker/NUC/vr

source docker_join.sh


## Docker

cd GMU_NCTU_Unity_ROS

load_pyrobot_env

source environment.sh

rosrun oculusVR pub_locobot_tf.py



## Before running this DO STEP 1 IN UNITY (Below)
# 6. control arm by VR controller( current state : control robot_arm position)
ssh locobot@locobotgmu.local

# NUC

cd GMU_NCTU_Unity_ROS/Docker/NUC/vr

source docker_join.sh

# Docker

cd ~/GMU_NCTU_Unity_ROS/ROS/catkin_ws/src/oculusVR/src

### PLEASE MAKE SURE TO chmod +x test_arm.py BEFORE you run the first time.

cd GMU_NCTU_Unity_ROS

load_pyrobot_env

source environment.sh

rosrun oculusVR test_arm.py


# IN Unity:
 
After first player has joined:

	1. Fake locobot(Clone) will spawn.

	2. Drag the “Fake locobot(Clone)” to “Robot Connector (Fake)” in Urdf Robot

	3. Run Step #6

	4. Press “Enable” Publish Joint State

# To control, hold down the left or right grip button while moving the right controller. 
