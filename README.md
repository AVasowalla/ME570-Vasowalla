# ME570-Vasowalla
 ME 570 Robot Motion Planning Coursework

colcon build --symlink-install
[0.354s] WARNING:colcon.colcon_core.package_identification:Failed to parse ROS package manifest in 'src/bearing_formation_control': Error(s) in package 'src/bearing_formation_control/package.xml':
Error(s):
- The generic dependency on 'rclpy' is redundant with: exec_depend
Starting >>> bearing_formation_control
--- stderr: bearing_formation_control                         
Error parsing '/home/armaan/me570-final/src/bearing_formation_control/package.xml':
Traceback (most recent call last):
  File "/opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 150, in <module>
    main()
  File "/opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 53, in main
    raise e
  File "/opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py", line 49, in main
    package = parse_package_string(
  File "/usr/lib/python3/dist-packages/catkin_pkg/package.py", line 786, in parse_package_string
    raise InvalidPackage('Error(s):%s' % (''.join(['\n- %s' % e for e in errors])), filename)
catkin_pkg.package.InvalidPackage: Error(s) in package '/home/armaan/me570-final/src/bearing_formation_control/package.xml':
Error(s):
- The generic dependency on 'rclpy' is redundant with: exec_depend
CMake Error at /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package_xml.cmake:95 (message):
  execute_process(/usr/bin/python3
  /opt/ros/humble/share/ament_cmake_core/cmake/core/package_xml_2_cmake.py
  /home/armaan/me570-final/src/bearing_formation_control/package.xml
  /home/armaan/me570-final/build/bearing_formation_control/ament_cmake_core/package.cmake)
  returned error code 1
Call Stack (most recent call first):
  /opt/ros/humble/share/ament_cmake_core/cmake/core/ament_package_xml.cmake:49 (_ament_package_xml)
  /opt/ros/humble/share/rosidl_cmake/cmake/rosidl_generate_interfaces.cmake:226 (ament_package_xml)
  CMakeLists.txt:21 (rosidl_generate_interfaces)


---
Failed   <<< bearing_formation_control [0.77s, exited with code 1]

Summary: 0 packages finished [1.13s]
  1 package failed: bearing_formation_control
  1 package had stderr output: bearing_formation_control
