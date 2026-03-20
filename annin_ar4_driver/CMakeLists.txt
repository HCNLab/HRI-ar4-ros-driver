cmake_minimum_required(VERSION 3.5)
project(annin_ar4_driver)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(rosidl_default_generators REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteRobotCommand.srv"
)

set(ANNIN_AR4_DRIVER_LIB_TARGET annin_ar4_driver_hw)

add_library(${ANNIN_AR4_DRIVER_LIB_TARGET} SHARED 
  src/ar_hardware_interface.cpp
  src/ar_servo_gripper_hw_interface.cpp
  src/teensy_driver.cpp
  src/arduino_nano_driver.cpp
  src/gripper_overcurrent_protection.cpp
)
set_target_properties(${ANNIN_AR4_DRIVER_LIB_TARGET} PROPERTIES OUTPUT_NAME ${PROJECT_NAME})

target_compile_features(${ANNIN_AR4_DRIVER_LIB_TARGET} PUBLIC cxx_std_17)
target_include_directories(${ANNIN_AR4_DRIVER_LIB_TARGET} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${ANNIN_AR4_DRIVER_LIB_TARGET}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
  Boost
)

add_executable(run_arduino_nano src/run_arduino_nano.cpp)

target_link_libraries(
  run_arduino_nano
  ${ANNIN_AR4_DRIVER_LIB_TARGET}
)

install(TARGETS
  run_arduino_nano
  DESTINATION lib/${PROJECT_NAME}
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)


install(TARGETS ${ANNIN_AR4_DRIVER_LIB_TARGET}
  EXPORT export_annin_ar4_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/reset_estop.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_targets(export_annin_ar4_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS} Boost)
ament_export_dependencies(rosidl_default_runtime)

ament_package()
