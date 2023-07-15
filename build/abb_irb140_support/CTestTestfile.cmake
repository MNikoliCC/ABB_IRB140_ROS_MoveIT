# CMake generated Testfile for 
# Source directory: /home/pilaciv/Workspaces/abb_irb140_ws/src/abb_irb140_support
# Build directory: /home/pilaciv/Workspaces/abb_irb140_ws/build/abb_irb140_support
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(_ctest_abb_irb140_support_roslaunch-check_tests_roslaunch_test_140.xml "/home/pilaciv/Workspaces/abb_irb140_ws/build/abb_irb140_support/catkin_generated/env_cached.sh" "/usr/bin/python3" "/opt/ros/noetic/share/catkin/cmake/test/run_tests.py" "/home/pilaciv/Workspaces/abb_irb140_ws/build/abb_irb140_support/test_results/abb_irb140_support/roslaunch-check_tests_roslaunch_test_140.xml.xml" "--return-code" "/usr/bin/cmake -E make_directory /home/pilaciv/Workspaces/abb_irb140_ws/build/abb_irb140_support/test_results/abb_irb140_support" "/opt/ros/noetic/share/roslaunch/cmake/../scripts/roslaunch-check -o \"/home/pilaciv/Workspaces/abb_irb140_ws/build/abb_irb140_support/test_results/abb_irb140_support/roslaunch-check_tests_roslaunch_test_140.xml.xml\" \"/home/pilaciv/Workspaces/abb_irb140_ws/src/abb_irb140_support/tests/roslaunch_test_140.xml\" ")
set_tests_properties(_ctest_abb_irb140_support_roslaunch-check_tests_roslaunch_test_140.xml PROPERTIES  _BACKTRACE_TRIPLES "/opt/ros/noetic/share/catkin/cmake/test/tests.cmake;160;add_test;/opt/ros/noetic/share/roslaunch/cmake/roslaunch-extras.cmake;66;catkin_run_tests_target;/home/pilaciv/Workspaces/abb_irb140_ws/src/abb_irb140_support/CMakeLists.txt;20;roslaunch_add_file_check;/home/pilaciv/Workspaces/abb_irb140_ws/src/abb_irb140_support/CMakeLists.txt;0;")
subdirs("gtest")
