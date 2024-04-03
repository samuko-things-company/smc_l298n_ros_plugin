cmake_minimum_required(VERSION 3.16)
project(smc_ros_hw_plugin LANGUAGES CXX)

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
  smc_ros_hw_plugin
  SHARED
  hardware/smc_ros_hw_plugin.cpp
)
target_compile_features(smc_ros_hw_plugin PUBLIC cxx_std_17)
target_include_directories(smc_ros_hw_plugin PUBLIC
$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
$<INSTALL_INTERFACE:include/smc_ros_hw_plugin>
)
ament_target_dependencies(
  smc_ros_hw_plugin PUBLIC
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(smc_ros_hw_plugin PUBLIC serial)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "SMC_ROS_HW_PLUGIN_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface smc_ros_hw_plugin.xml)

# INSTALL
install(
  DIRECTORY hardware/include/
  DESTINATION include/smc_ros_hw_plugin
)

install(TARGETS smc_ros_hw_plugin
  EXPORT export_smc_ros_hw_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_targets(export_smc_ros_hw_plugin HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
