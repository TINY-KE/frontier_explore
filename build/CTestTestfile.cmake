# CMake generated Testfile for 
# Source directory: /home/zhjd/active_ws/src/m-explore/explore
# Build directory: /home/zhjd/active_ws/src/m-explore/explore/build
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_explore_lite_roslaunch-check_launch "/home/zhjd/active_ws/src/m-explore/explore/build/catkin_generated/env_cached.sh" "/usr/bin/python2" "/opt/ros/kinetic/share/catkin/cmake/test/run_tests.py" "/home/zhjd/active_ws/src/m-explore/explore/build/test_results/explore_lite/roslaunch-check_launch.xml" "--return-code" "/usr/local/bin/cmake -E make_directory /home/zhjd/active_ws/src/m-explore/explore/build/test_results/explore_lite" "/opt/ros/kinetic/share/roslaunch/cmake/../scripts/roslaunch-check -o '/home/zhjd/active_ws/src/m-explore/explore/build/test_results/explore_lite/roslaunch-check_launch.xml' '/home/zhjd/active_ws/src/m-explore/explore/launch' ")
set_tests_properties(_ctest_explore_lite_roslaunch-check_launch PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/kinetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/kinetic/share/roslaunch/cmake/roslaunch-extras.cmake;58;catkin_run_tests_target;/home/zhjd/active_ws/src/m-explore/explore/CMakeLists.txt;82;roslaunch_add_file_check;/home/zhjd/active_ws/src/m-explore/explore/CMakeLists.txt;0;")
subdirs("gtest")
