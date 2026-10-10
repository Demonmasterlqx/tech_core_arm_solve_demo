cmake_minimum_required(VERSION 3.8)
project(handle_arm)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  sensor_msgs
  std_msgs
  trajectory_msgs
  yaml_cpp_vendor
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/trajectory_loop_node.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    "$<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

target_link_libraries(${PROJECT_NAME}
  yaml-cpp::yaml-cpp
)

ament_target_dependencies(${PROJECT_NAME}
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

add_executable(handle_arm_trajectory_loop_node
  src/main.cpp
)

target_link_libraries(handle_arm_trajectory_loop_node
  ${PROJECT_NAME}
)

ament_target_dependencies(handle_arm_trajectory_loop_node
  rclcpp
)

install(
  TARGETS
    ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS handle_arm_trajectory_loop_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_trajectory_loop_node
    test/test_trajectory_loop_node.cpp
  )
  if(TARGET test_trajectory_loop_node)
    target_compile_features(test_trajectory_loop_node PUBLIC cxx_std_17)
    target_link_libraries(test_trajectory_loop_node ${PROJECT_NAME})
    ament_target_dependencies(test_trajectory_loop_node
      rclcpp
      sensor_msgs
      std_msgs
      trajectory_msgs
    )
  endif()
endif()

ament_package()
