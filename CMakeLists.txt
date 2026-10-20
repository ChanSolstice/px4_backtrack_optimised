cmake_minimum_required(VERSION 3.0.2)
project(px4_backtrack_optimised)

## C++17 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## 编译选项
add_compile_options(-Wall -Wextra)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  mavros_msgs
  visualization_msgs
  trajectory_core
)

## Find Eigen3 (trajectory_core依赖)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    geometry_msgs
    mavros_msgs
    visualization_msgs
    trajectory_core
  DEPENDS
    EIGEN3
)

###########
## Build ##
###########

## Include directories
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

## Domain层源文件
set(DOMAIN_SOURCES
  src/domain/state_machine.cpp
  src/domain/recording_buffer.cpp
  src/domain/trigger_evaluator.cpp
  src/domain/path_yaw.cpp
  src/domain/trajectory_session.cpp
  src/domain/waypoint_fallback_session.cpp
  src/domain/rejoin_policy.cpp
  src/domain/link_monitor.cpp
  src/domain/pilot_input.cpp
  src/domain/battery_monitor.cpp
)

## Application层源文件
set(APPLICATION_SOURCES
  src/application/backtrack_usecase.cpp
)

## Adapters层源文件
set(ADAPTERS_SOURCES
  src/adapters/ros_subscribers.cpp
  src/adapters/ros_publishers.cpp
  src/adapters/ros_services.cpp
  src/adapters/debug_publishers.cpp
)

## Infrastructure层源文件
set(INFRASTRUCTURE_SOURCES
  src/infrastructure/config_loader.cpp
  src/infrastructure/debug_dumper.cpp
)

## 所有库源文件
set(LIB_SOURCES
  ${DOMAIN_SOURCES}
  ${APPLICATION_SOURCES}
  ${ADAPTERS_SOURCES}
  ${INFRASTRUCTURE_SOURCES}
)

## 构建库（可被主节点和测试使用）
add_library(${PROJECT_NAME}_lib
  ${LIB_SOURCES}
)

add_dependencies(${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  stdc++fs
)

## 构建主节点可执行文件
add_executable(backtrack_node
  src/backtrack_node.cpp
)

add_dependencies(backtrack_node
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(backtrack_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Install executables
install(TARGETS backtrack_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install library
install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Install header files
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

## Install launch files
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch"
)

## Install config files
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_recording_buffer
    tests/unit/test_main.cpp
    tests/unit/test_recording_buffer.cpp
  )
  if(TARGET test_recording_buffer)
    target_link_libraries(test_recording_buffer
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_trigger_evaluator
    tests/unit/test_main.cpp
    tests/unit/test_trigger_evaluator.cpp
  )
  if(TARGET test_trigger_evaluator)
    target_link_libraries(test_trigger_evaluator
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_link_monitor
    tests/unit/test_main.cpp
    tests/unit/test_link_monitor.cpp
  )
  if(TARGET test_link_monitor)
    target_link_libraries(test_link_monitor
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_pilot_input
    tests/unit/test_main.cpp
    tests/unit/test_pilot_input.cpp
  )
  if(TARGET test_pilot_input)
    target_link_libraries(test_pilot_input
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_battery_monitor
    tests/unit/test_main.cpp
    tests/unit/test_battery_monitor.cpp
  )
  if(TARGET test_battery_monitor)
    target_link_libraries(test_battery_monitor
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_state_machine
    tests/unit/test_main.cpp
    tests/unit/test_state_machine.cpp
  )
  if(TARGET test_state_machine)
    target_link_libraries(test_state_machine
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_ros_subscribers
    tests/unit/test_main.cpp
    tests/unit/test_ros_subscribers.cpp
  )
  if(TARGET test_ros_subscribers)
    target_link_libraries(test_ros_subscribers
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_path_yaw
    tests/unit/test_main.cpp
    tests/unit/test_path_yaw.cpp
  )
  if(TARGET test_path_yaw)
    target_link_libraries(test_path_yaw
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_rejoin_policy
    tests/unit/test_main.cpp
    tests/unit/test_rejoin_policy.cpp
  )
  if(TARGET test_rejoin_policy)
    target_link_libraries(test_rejoin_policy
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_trajectory_session
    tests/unit/test_main.cpp
    tests/unit/test_trajectory_session.cpp
  )
  if(TARGET test_trajectory_session)
    target_link_libraries(test_trajectory_session
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_waypoint_fallback_session
    tests/unit/test_main.cpp
    tests/unit/test_waypoint_fallback_session.cpp
  )
  if(TARGET test_waypoint_fallback_session)
    target_link_libraries(test_waypoint_fallback_session
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_backtrack_usecase
    tests/unit/test_main.cpp
    tests/unit/test_backtrack_usecase.cpp
  )
  if(TARGET test_backtrack_usecase)
    target_link_libraries(test_backtrack_usecase
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()
endif()
