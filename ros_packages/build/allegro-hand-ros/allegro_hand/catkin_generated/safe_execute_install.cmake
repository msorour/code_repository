execute_process(COMMAND "/home/work/code_repository/ros_packages/build/allegro-hand-ros/allegro_hand/catkin_generated/python_distutils_install.sh" RESULT_VARIABLE res)

if(NOT res EQUAL 0)
  message(FATAL_ERROR "execute_process(/home/work/code_repository/ros_packages/build/allegro-hand-ros/allegro_hand/catkin_generated/python_distutils_install.sh) returned error code ")
endif()
