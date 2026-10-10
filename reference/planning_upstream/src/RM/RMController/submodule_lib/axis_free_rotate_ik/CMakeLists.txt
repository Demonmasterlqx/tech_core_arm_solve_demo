cmake_minimum_required(VERSION 3.8)
project(axis_free_rotate_ik)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
find_package(generate_parameter_library REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pinocchio REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(trajectory_msgs REQUIRED)

set(THIS_PACKAGE_DEPENDENCIES
  geometry_msgs
  rclcpp
  rclcpp_components
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SolveFixedIK.srv"
  "srv/SolveAxisFreeIK.srv"
  DEPENDENCIES geometry_msgs trajectory_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

generate_parameter_library(axis_free_rotate_ik_ros_parameters
  src/axis_free_rotate_ik_ros.yaml
)

add_library(axis_free_rotate_ik_core SHARED
  src/axis_free_rotate_ik.cpp
)

set_target_properties(axis_free_rotate_ik_core PROPERTIES
  OUTPUT_NAME axis_free_rotate_ik
)

target_compile_features(axis_free_rotate_ik_core PUBLIC cxx_std_17)
target_include_directories(axis_free_rotate_ik_core PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
target_link_libraries(axis_free_rotate_ik_core PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
)

add_library(axis_free_rotate_ik_ros_component SHARED
  src/axis_free_rotate_ik_ros.cpp
)
target_compile_features(axis_free_rotate_ik_ros_component PUBLIC cxx_std_17)
target_include_directories(axis_free_rotate_ik_ros_component PUBLIC
  "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include>"
)
ament_target_dependencies(axis_free_rotate_ik_ros_component
  ${THIS_PACKAGE_DEPENDENCIES}
)
target_link_libraries(axis_free_rotate_ik_ros_component
  axis_free_rotate_ik_core
  axis_free_rotate_ik_ros_parameters
  "${cpp_typesupport_target}"
)

rclcpp_components_register_node(axis_free_rotate_ik_ros_component
  PLUGIN "axis_free_rotate_ik::AxisFreeRotateIKRos"
  EXECUTABLE axis_free_rotate_ik_ros_node
)

ament_export_include_directories(include)
ament_export_dependencies(
  Eigen3
  generate_parameter_library
  geometry_msgs
  pinocchio
  rclcpp
  rclcpp_components
  rosidl_default_runtime
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  trajectory_msgs
)
ament_export_targets(export_axis_free_rotate_ik HAS_LIBRARY_TARGET)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  TARGETS
    axis_free_rotate_ik_core
    axis_free_rotate_ik_ros_component
    axis_free_rotate_ik_ros_parameters
  EXPORT export_axis_free_rotate_ik
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(FILES src/axis_free_rotate_ik_ros.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_axis_free_rotate_ik
    test/test_axis_free_rotate_ik.cpp
  )
  target_link_libraries(test_axis_free_rotate_ik
    axis_free_rotate_ik_core
    Eigen3::Eigen
    pinocchio::pinocchio
  )

  ament_add_gtest(test_axis_free_rotate_ik_ros
    test/test_axis_free_rotate_ik_ros.cpp
  )
  if(TARGET test_axis_free_rotate_ik_ros)
    target_link_libraries(test_axis_free_rotate_ik_ros
      axis_free_rotate_ik_ros_component
      "${cpp_typesupport_target}"
    )
    ament_target_dependencies(test_axis_free_rotate_ik_ros
      geometry_msgs
      rclcpp
      std_msgs
      tf2
      tf2_ros
      trajectory_msgs
    )
  endif()

  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_ADDITIONAL_ARGS
    CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/uncrustify_4_spaces.cfg"
  )
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
