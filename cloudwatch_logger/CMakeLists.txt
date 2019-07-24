cmake_minimum_required(VERSION 2.8.3)
project(cloudwatch_logger)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Enable strict compiler flags if possible.
include(CheckCXXCompilerFlag)
# Removed -Wmissing-declarations until gmock is ignored
# removed -Werror=pedantic until ros.h is fixed
set(FLAGS -Wno-long-long -Wall -Wextra -Wcast-align -Wcast-qual -Wformat -Wwrite-strings)
foreach(FLAG ${FLAGS})
  check_cxx_compiler_flag(${FLAG} R${FLAG})
  if(${R${FLAG}})
    set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${FLAG}")
  endif()
endforeach()

if(NOT DEFINED CXX_DISABLE_WERROR)
  set(WARNING_CXX_FLAGS "-Werror ${WARNING_CXX_FLAGS}")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${WARNING_CXX_FLAGS}")

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)

find_package(dataflow_lite REQUIRED)
find_package(file_management REQUIRED)
find_package(cloudwatch_logs_common REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
    cloudwatch_logs_common
    roscpp
    aws_common
    aws_ros1_common
    rosgraph_msgs
    std_srvs
    std_msgs)

###################################
## catkin specific configuration ##
###################################

catkin_package()

###########
## Build ##
###########

## Declare a C++ executable
add_library(${PROJECT_NAME}_lib SHARED src/log_node.cpp src/log_node_param_helper.cpp)

target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${dataflow_lite_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${cloudwatch_logs_common_LIBRARIES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${file_management_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${file_management_LIBRARIES})

target_include_directories(${PROJECT_NAME}_lib PUBLIC include ${catkin_INCLUDE_DIRS} ${cloudwatch_logs_common_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}_lib ${catkin_LIBRARIES} ${cloudwatch_logs_common_LIBRARIES})

add_executable(${PROJECT_NAME} src/main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include ${catkin_INCLUDE_DIRS} ${cloudwatch_logs_common_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${PROJECT_NAME}_lib ${cloudwatch_logs_common_LIBRARIES})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
        
install(DIRECTORY config launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  find_package(GMock QUIET)
  if(GMOCK_FOUND)
    add_rostest_gmock(test_log_node
      test/test_log_node.test
      test/log_node_test.cpp
    )
    target_include_directories(test_log_node PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${cloudwatch_logger_INCLUDE_DIRS}
      ${GMOCK_INCLUDE_DIRS}
    )
    target_link_libraries(test_log_node
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${GMOCK_BOTH_LIBRARIES}
    )

    add_rostest_gmock(test_log_node_param_helper
      test/test_log_node_param_helper.test
      test/log_node_param_helper_test.cpp
    )
    target_include_directories(test_log_node_param_helper PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${cloudwatch_logger_INCLUDE_DIRS}
      ${GMOCK_INCLUDE_DIRS}
    )
    target_link_libraries(test_log_node_param_helper
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${GMOCK_BOTH_LIBRARIES}
    )
  else()
    include_directories(/usr/include/gmock /usr/src/gmock)
    add_library(${PROJECT_NAME}_libgmock SHARED /usr/src/gmock/src/gmock-all.cc)

    add_rostest_gtest(test_log_node
      test/test_log_node.test
      test/log_node_test.cpp
    )
    target_include_directories(test_log_node PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${cloudwatch_logger_INCLUDE_DIRS}
    )
    target_link_libraries(test_log_node
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}_libgmock
    )

    add_rostest_gtest(test_log_node_param_helper
      test/test_log_node_param_helper.test
      test/log_node_param_helper_test.cpp
    )
    target_include_directories(test_log_node_param_helper PRIVATE
      include
      ${catkin_INCLUDE_DIRS}
      ${cloudwatch_logger_INCLUDE_DIRS}
    )
    target_link_libraries(test_log_node_param_helper
      ${PROJECT_NAME}_lib
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}_libgmock
    )
  endif()
endif()
