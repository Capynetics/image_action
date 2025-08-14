cmake_minimum_required(VERSION 3.8)
project(image_action_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Generate action interfaces (use a unique target name to avoid clashes)
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/CompareImages.action"
  DEPENDENCIES sensor_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
