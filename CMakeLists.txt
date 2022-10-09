cmake_minimum_required(VERSION 3.8)
project(modbus)

# Compiler settings
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

# Logic shared by all targets
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ConfiguredHoldingRegisterRead.srv"
  "srv/ConfiguredHoldingRegisterWrite.srv"
  "srv/HoldingRegisterRead.srv"
  "srv/HoldingRegisterWrite.srv"
  "srv/HoldingRegisterWriteMultiple.srv"
  "srv/GetComEventLog.srv"
  "srv/ReadDeviceId.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

include_directories(
  include
  ${YAML_CPP_INCLUDEDIR}
)

set(project_SOURCE_FILES
  src/interface.cpp
  src/interface_remote.cpp
  src/implementation.cpp
  src/proto_holding_registers.cpp
  src/proto_misc.cpp
)

set(project_LIBRARIES
  ${rclcpp_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

set(project_DEPENDENCIES
  rclcpp
  std_msgs
  yaml-cpp
)

# Executable target
add_executable(${PROJECT_NAME}_standalone src/main.cpp ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_standalone ${project_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_standalone ${project_DEPENDENCIES})
install(TARGETS
  ${PROJECT_NAME}_standalone
  DESTINATION lib
)

# Library target
add_library(${PROJECT_NAME}_native src/factory.cpp ${project_SOURCE_FILES})
target_link_libraries(${PROJECT_NAME}_native ${project_LIBRARIES} "${cpp_typesupport_target}")
ament_target_dependencies(${PROJECT_NAME}_native ${project_DEPENDENCIES})
ament_export_targets(${PROJECT_NAME}_native_library HAS_LIBRARY_TARGET)
ament_export_dependencies(${project_DEPENDENCIES})
ament_export_include_directories(include)
install(TARGETS ${PROJECT_NAME}_native
  EXPORT ${PROJECT_NAME}_native_library
  LIBRARY DESTINATION lib
)
install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
