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
  message_generation
  roslib
  tf2
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
find_package(Threads REQUIRED)

###################################
## catkin specific configuration ##
###################################
add_message_files(FILES ControlDiagnostics.msg PlanningTrace.msg)
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    message_runtime
    roscpp
    roslib
    tf2
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
  src/domain/recorded_route.cpp
)

## Application层源文件
set(APPLICATION_SOURCES
  src/application/backtrack_usecase.cpp
  src/application/planning_worker.cpp
)

## Adapters层源文件
set(ADAPTERS_SOURCES
  src/adapters/ros_subscribers.cpp
  src/adapters/ros_publishers.cpp
  src/adapters/ros_services.cpp
  src/adapters/debug_publishers.cpp
  src/adapters/diagnostics_publisher.cpp
  src/adapters/planning_diagnostics.cpp
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
  Threads::Threads
)

# The recorded revision describes the built sources, not a later runtime checkout.
set(BACKTRACK_BUILD_REVISION "" CACHE STRING "Revision embedded in diagnostic metadata")
if(NOT BACKTRACK_BUILD_REVISION)
  execute_process(COMMAND git describe --always --dirty
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE BACKTRACK_BUILD_REVISION OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(NOT BACKTRACK_BUILD_REVISION)
    set(BACKTRACK_BUILD_REVISION "unknown")
  endif()
endif()

## 构建主节点可执行文件
add_executable(backtrack_node
  src/backtrack_node.cpp
)

add_dependencies(backtrack_node
  ${PROJECT_NAME}_lib
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_compile_definitions(${PROJECT_NAME}_lib PRIVATE BACKTRACK_BUILD_REVISION="${BACKTRACK_BUILD_REVISION}")

target_link_libraries(backtrack_node
  ${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

## Install executables
catkin_install_python(PROGRAMS scripts/record_flight_log.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

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
install(DIRECTORY include/px4_backtrack/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/px4_backtrack
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
  catkin_add_nosetests(tests/unit/test_flight_log_recorder.py)
  catkin_add_gtest(test_control_diagnostics tests/unit/test_main.cpp tests/unit/test_control_diagnostics.cpp)
  if(TARGET test_control_diagnostics)
    target_link_libraries(test_control_diagnostics ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
  endif()
  catkin_add_gtest(test_async_control tests/unit/test_main.cpp tests/unit/test_async_control.cpp)
  if(TARGET test_async_control)
    target_link_libraries(test_async_control ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
  endif()

  catkin_add_gtest(test_recorded_route tests/unit/test_main.cpp tests/unit/test_recorded_route.cpp)
  if(TARGET test_recorded_route)
    target_link_libraries(test_recorded_route ${PROJECT_NAME}_lib ${catkin_LIBRARIES})
  endif()

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
