cmake_minimum_required(VERSION 2.8.3)
project(intera_test_suite)
 add_compile_options(-std=c++11 -funroll-loops -Wall -Ofast)
 set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  subscription_notifier
  intera_core_msgs
  roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES intera_test_suite
  CATKIN_DEPENDS geometry_msgs sensor_msgs roscpp subscription_notifier intera_core_msgs
  DEPENDS
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

 add_executable(position_sine_wave src/position_sine_wave.cpp)
 add_dependencies(position_sine_wave ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
 target_link_libraries(position_sine_wave
   ${catkin_LIBRARIES}
 )

 install(TARGETS position_sine_wave
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
# Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
