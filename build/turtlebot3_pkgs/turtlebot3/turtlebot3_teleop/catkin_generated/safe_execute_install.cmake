execute_process(COMMAND "/home/ahmed/all_Practice/ros_diploma/AMR_Ros1_Projects/build/turtlebot3_pkgs/turtlebot3/turtlebot3_teleop/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/ahmed/all_Practice/ros_diploma/AMR_Ros1_Projects/build/turtlebot3_pkgs/turtlebot3/turtlebot3_teleop/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
