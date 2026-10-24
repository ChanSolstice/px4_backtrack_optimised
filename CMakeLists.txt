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
  src/domain/recording_buffer.cpp
  src/domain/path_yaw.cpp
  src/domain/trajectory_session.cpp
  src/domain/waypoint_fallback_session.cpp
  src/domain/rejoin_policy.cpp
  src/domain/link_monitor.cpp
  src/domain/pilot_input.cpp
  src/domain/battery_monitor.cpp
  src/domain/airborne_monitor.cpp
  src/domain/return_state_machine.cpp
)

## Application层源文件
set(APPLICATION_SOURCES
  src/application/backtrack_usecase.cpp
  src/application/mode_commander.cpp
  src/application/planning_worker.cpp
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
  src/infrastructure/debug_log.cpp
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

# 调试日志的 config 事件记录本模块的提交号（取不到时为 unknown）
execute_process(
  COMMAND git rev-parse --short HEAD
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE BACKTRACK_GIT_REV
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_QUIET
)
if(BACKTRACK_GIT_REV)
  target_compile_definitions(backtrack_node PRIVATE BACKTRACK_GIT_REV="${BACKTRACK_GIT_REV}")
endif()

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

  catkin_add_gtest(test_mode_commander
    tests/unit/test_main.cpp
    tests/unit/test_mode_commander.cpp
  )
  if(TARGET test_mode_commander)
    target_link_libraries(test_mode_commander
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_planning_worker
    tests/unit/test_main.cpp
    tests/unit/test_planning_worker.cpp
  )
  if(TARGET test_planning_worker)
    target_link_libraries(test_planning_worker
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_debug_log
    tests/unit/test_main.cpp
    tests/unit/test_debug_log.cpp
  )
  if(TARGET test_debug_log)
    target_link_libraries(test_debug_log
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_config_loader
    tests/unit/test_main.cpp
    tests/unit/test_config_loader.cpp
  )
  if(TARGET test_config_loader)
    target_link_libraries(test_config_loader
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
    target_compile_definitions(test_config_loader PRIVATE
      BACKTRACK_CONFIG_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config")
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

  catkin_add_gtest(test_airborne_monitor
    tests/unit/test_main.cpp
    tests/unit/test_airborne_monitor.cpp
  )
  if(TARGET test_airborne_monitor)
    target_link_libraries(test_airborne_monitor
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_return_state_machine
    tests/unit/test_main.cpp
    tests/unit/test_return_state_machine.cpp
  )
  if(TARGET test_return_state_machine)
    target_link_libraries(test_return_state_machine
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(test_return_state_machine_invariants
    tests/unit/test_main.cpp
    tests/unit/test_return_state_machine_invariants.cpp
  )
  if(TARGET test_return_state_machine_invariants)
    target_link_libraries(test_return_state_machine_invariants
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
