cmake_minimum_required(VERSION 3.16)
project(hardware_fcs LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()


## COMPILE
add_library(
  hardware_fcs
  SHARED
  hardware_fcs.cc
)
target_compile_features(hardware_fcs PUBLIC cxx_std_11)
target_include_directories(hardware_fcs PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/hardware_fcs>
)
ament_target_dependencies(
  hardware_fcs PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "HARDWARE_FCS_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_fcs.xml)

# INSTALL
install(
  DIRECTORY include/
  DESTINATION include/hardware_fcs
)
#install(
#  DIRECTORY description/launch description/ros2_control description/urdf
#  DESTINATION share/hardware_fcs
#)
#install(
#  DIRECTORY bringup/launch bringup/config
#  DESTINATION share/hardware_fcs
#)
install(TARGETS hardware_fcs
  EXPORT export_hardware_fcs
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_hardware_fcs HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
