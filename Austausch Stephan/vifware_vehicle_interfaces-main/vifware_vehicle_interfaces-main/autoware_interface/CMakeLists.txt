cmake_minimum_required(VERSION 3.8)
project(autoware_interface)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(autoware_adapi_v1_msgs REQUIRED)
find_package(autoware_control_msgs REQUIRED)
find_package(autoware_vehicle_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tier4_vehicle_msgs REQUIRED)
find_package(vifware_msgs REQUIRED)
find_package(vifware_vehicle_interface_common REQUIRED)
find_package(vifware_vehicle_interface_common REQUIRED)
find_package(vifware_vehicle_interface_msgs REQUIRED)
find_package(iceoryx_hoofs REQUIRED)

include_directories(include)

set(DEPS
  autoware_adapi_v1_msgs
  autoware_control_msgs
  autoware_vehicle_msgs
  rclcpp
  rclcpp_components
  tier4_vehicle_msgs
  vifware_msgs
  vifware_vehicle_interface_common
  vifware_vehicle_interface_msgs
  iceoryx_hoofs
  )


#####################
## autoware interface
add_library(autoware_interface_component SHARED
  src/autoware_interface.cpp
  src/autoware_ad_api_client.cpp
  )

ament_target_dependencies(autoware_interface_component ${DEPS})

install(
  TARGETS autoware_interface_component
  EXPORT autoware_interface_component
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(autoware_interface_component HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPS})


# #########################
# ## autoware ad api client
# add_library(autoware_ad_api_component SHARED
#   src/autoware_ad_api_client.cpp
#   )

# ament_target_dependencies(autoware_ad_api_component ${DEPS})

# rclcpp_components_register_nodes(autoware_ad_api_component
#   "vifware_vehicle_interfaces::autoware_ad_api_component"
#   )

# install(
#   TARGETS autoware_ad_api_component
#   EXPORT autoware_ad_api_component
#   LIBRARY DESTINATION lib
#   ARCHIVE DESTINATION lib
#   RUNTIME DESTINATION bin
#   INCLUDES DESTINATION include
# )

# ament_export_include_directories(include)
# ament_export_targets(autoware_ad_api_component HAS_LIBRARY_TARGET)
# ament_export_dependencies(${DEPS})

# install
install(DIRECTORY include/
  DESTINATION include/
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
