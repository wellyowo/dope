execute_process(COMMAND "/home/dope/dope/catkin_ws/build/Deep_Object_Pose/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/dope/dope/catkin_ws/build/Deep_Object_Pose/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
