# CMake generated Testfile for 
# Source directory: /home/laboratorio/ros_workspace/src/proyecto_final
# Build directory: /home/laboratorio/ros_workspace/build/proyecto_final
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_proyecto_final_roslaunch-check_launch "/home/laboratorio/ros_workspace/build/proyecto_final/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/laboratorio/ros_workspace/build/proyecto_final/test_results/proyecto_final/roslaunch-check_launch.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/laboratorio/ros_workspace/build/proyecto_final/test_results/proyecto_final" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/laboratorio/ros_workspace/build/proyecto_final/test_results/proyecto_final/roslaunch-check_launch.xml\" \"/home/laboratorio/ros_workspace/src/proyecto_final/launch\" ")
set_tests_properties(_ctest_proyecto_final_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/laboratorio/ros_workspace/src/proyecto_final/CMakeLists.txt;51;roslaunch_add_file_check;/home/laboratorio/ros_workspace/src/proyecto_final/CMakeLists.txt;0;")
subdirs("gtest")
