# ME570-Vasowalla
 ME 570 Robot Motion Planning Coursework

cmake_minimum_required(VERSION 3.5)
project(bearing_formation_control)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(turtlesim REQUIRED)

# Add message generation
find_package(rosidl_default_generators REQUIRED)

# Add your message files here
set(msg_files
    "msg/command.msg"
    "msg/targets.msg"
    "msg/sensordata.msg"
)

# Generate the messages
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs  # Add other dependencies here if needed
)

# Include directories for your Python code
include_directories(
    include
)

# Install Python files
install(PROGRAMS
    scripts/spawn_turtle.py
    scripts/command_executor.py
    scripts/logic.py
    DESTINATION lib/${PROJECT_NAME}
)

# Install your generated messages
install(DIRECTORY
    msg/
    DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
)

# Setup for ament
ament_package()
