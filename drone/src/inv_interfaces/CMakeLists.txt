cmake_minimum_required(VERSION 3.16)
project(inv_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  # Perception / inventory
  "msg/LabelDetection.msg"
  "msg/LabelDetectionArray.msg"
  "msg/InventoryItem.msg"
  "msg/InventoryItemArray.msg"
  # Health of each subsystem
  "msg/LocalizationStatus.msg"
  "msg/CorridorStatus.msg"
  "msg/FlightState.msg"
  "msg/SafetyStatus.msg"
  "msg/MissionStatus.msg"
  # Commands
  "srv/Abort.srv"
  "srv/ExportCsv.srv"
  "action/Takeoff.action"
  "action/GoTo.action"
  "action/Hover.action"
  "action/Land.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  # Unit test: every interface imports and its constants match docs/interfaces.md.
  ament_add_pytest_test(test_interfaces test/test_interfaces.py TIMEOUT 60)
endif()

ament_package()
