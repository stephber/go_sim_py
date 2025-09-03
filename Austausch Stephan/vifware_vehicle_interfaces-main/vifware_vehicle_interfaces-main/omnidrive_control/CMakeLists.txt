cmake_minimum_required(VERSION 3.16)
project(omnidrive_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  geometry_msgs
  nav_msgs
  vifware_vehicle_interface_common
  rclcpp
  rclcpp_components
  rcpputils
  realtime_tools
  iceoryx_hoofs  
  tf2
  tf2_msgs
  iceoryx_hoofs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Add omnidrive_control library related compile commands
generate_parameter_library(${PROJECT_NAME}_parameters
  src/omnidrive_control_parameter.yaml
  )

add_library(${PROJECT_NAME}_component SHARED
  src/omnidrive_control.cpp
  src/wheel.cpp
  src/odometry.cpp
  )

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

target_link_libraries(${PROJECT_NAME}_component
  ${PROJECT_NAME}_parameters
  )
ament_target_dependencies(${PROJECT_NAME}_component ${THIS_PACKAGE_INCLUDE_DEPENDS})


rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "vifware_vehicle_interfaces::${PROJECT_NAME}_component"
  )

# install
install(DIRECTORY include/
  DESTINATION include/
  )

#
install(
  TARGETS ${PROJECT_NAME}_component ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# export
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
#ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPEND})

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()


