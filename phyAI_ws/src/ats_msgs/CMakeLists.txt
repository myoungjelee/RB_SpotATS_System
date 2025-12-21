cmake_minimum_required(VERSION 3.8)
project(ats_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# msg에서 참조하는 패키지는 여기서도 find_package
find_package(builtin_interfaces REQUIRED)
# find_package(std_msgs REQUIRED)
# find_package(geometry_msgs REQUIRED)
# find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AtsState.msg"
  "msg/PlanCommand.msg"
  "msg/ReplanRequest.msg"
  DEPENDENCIES
    builtin_interfaces
    # std_msgs
    # geometry_msgs
    # sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
