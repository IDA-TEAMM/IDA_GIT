# CMake generated Testfile for 
# Source directory: /root/ros2_ws/src/ament/ament_cmake/ament_cmake_pytest
# Build directory: /root/ros2_ws/build/ament_cmake_pytest
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(pytest "/usr/bin/python3" "-u" "/root/ros2_ws/install/ament_cmake_test/share/ament_cmake_test/cmake/run_test.py" "/root/ros2_ws/build/ament_cmake_pytest/test_results/ament_cmake_pytest/pytest.xunit.xml" "--package-name" "ament_cmake_pytest" "--output-file" "/root/ros2_ws/build/ament_cmake_pytest/ament_cmake_pytest/pytest.txt" "--command" "/usr/bin/python3" "-u" "-m" "pytest" "/root/ros2_ws/src/ament/ament_cmake/ament_cmake_pytest/test" "-o" "cache_dir=/root/ros2_ws/build/ament_cmake_pytest/ament_cmake_pytest/pytest/.cache" "-s" "--junit-xml=/root/ros2_ws/build/ament_cmake_pytest/test_results/ament_cmake_pytest/pytest.xunit.xml" "--junit-prefix=ament_cmake_pytest")
set_tests_properties(pytest PROPERTIES  LABELS "pytest" TIMEOUT "60" WORKING_DIRECTORY "/root/ros2_ws/build/ament_cmake_pytest" _BACKTRACE_TRIPLES "/root/ros2_ws/install/ament_cmake_test/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/root/ros2_ws/src/ament/ament_cmake/ament_cmake_pytest/CMakeLists.txt;50;ament_add_test;/root/ros2_ws/src/ament/ament_cmake/ament_cmake_pytest/CMakeLists.txt;0;")
