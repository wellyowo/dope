#catkin_make ignore
touch ./ROS/catkin_ws/src/interbotix_ros_arms/CATKIN_IGNORE

# python3 openCV 
#catkin_make --pkg vision_opencv -C ./ROS/catkin_ws \
#    -DCMAKE_BUILD_TYPE=Release \
#    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
#    -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
#    -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

# python3 tf
#catkin_make --pkg geometry2 -C ./ROS/catkin_ws \
#	    --cmake-args \
#            -DCMAKE_BUILD_TYPE=Release \
#            -DPYTHON_EXECUTABLE=/usr/bin/python3 \
#            -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m \
#            -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so

catkin_make -C ./ROS/catkin_ws

