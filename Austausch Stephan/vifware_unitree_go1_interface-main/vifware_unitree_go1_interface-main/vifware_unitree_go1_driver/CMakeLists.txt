cmake_minimum_required(VERSION 3.8)
project(vifware_unitree_go1_driver)

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
  rclcpp
  rclcpp_components
  # vifware dependencies
  autoware_interface
  tinyfsm_ros2
  vifware_vehicle_interface_msgs
  vifware_vehicle_interface_common
  vifware_unitree_go1_msgs
  ros2_unitree_legged_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  )

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(generate_parameter_library REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


add_library(${PROJECT_NAME}_component SHARED
  src/unitree_go1_driver.cpp
  )

target_include_directories(${PROJECT_NAME}_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
  )

ament_target_dependencies(${PROJECT_NAME}_component ${THIS_PACKAGE_INCLUDE_DEPENDS})

rclcpp_components_register_nodes(${PROJECT_NAME}_component
  "vifware_unitree_go1_interface::${PROJECT_NAME}_component"
  )
# install
install(DIRECTORY include/
  DESTINATION include/
  )

install(
  TARGETS ${PROJECT_NAME}_component # ${PROJECT_NAME}_parameters
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
  )

# export
ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

## node for spider interface
add_executable(${PROJECT_NAME}
  src/unitree_go1_driver_node.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_component
  #omnidrive_controller
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  )

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
  )

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
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
