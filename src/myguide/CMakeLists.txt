cmake_minimum_required(VERSION 3.0.2)
project(myguide)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_filters
  pozyx_simulation
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES inertialAlign
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

# 设置Eigen库的路径
set(UWB_INCLUDE_DIR "/home/ouyy/MyRosGuideCar/devel/include/pozyx_simulation")
# ${EIGEN3_INCLUDE_DIR}

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${UWB_INCLUDE_DIR}
)

#声明c++库
add_library(${PROJECT_NAME}
  src/KFApp.cpp
  src/PSINS.cpp
  src/EXGUIDE.cpp
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#可执行文件
add_executable(imu_gps_gui src/imu_gps_gui.cpp)
add_executable(getrealposition src/getrealposition.cpp)
add_executable(imu_gps_gui_justgps src/imu_gps_gui_justgps.cpp)
add_executable(planningtest src/planningtest.cpp)
add_executable(uwbposition src/uwbposition.cpp)
add_executable(imuSolve src/imuSolve.cpp)
add_executable(eskftest src/eskftest.cpp)

add_dependencies(imu_gps_gui ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(getrealposition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imu_gps_gui_justgps ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(planningtest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(uwbposition ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(imuSolve ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(eskftest ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
## Specify libraries to link a library or executable target against

target_link_libraries(imu_gps_gui
${PROJECT_NAME} #使用头文件的依赖库
${catkin_LIBRARIES}
)

target_link_libraries(getrealposition
${PROJECT_NAME} #使用头文件的依赖库
${catkin_LIBRARIES}
)
target_link_libraries(imu_gps_gui_justgps
${PROJECT_NAME} #使用头文件的依赖库
${catkin_LIBRARIES}
)
target_link_libraries(planningtest
${PROJECT_NAME} #使用头文件的依赖库
${catkin_LIBRARIES}
)
target_link_libraries(uwbposition
${PROJECT_NAME} #使用头文件的依赖库
${catkin_LIBRARIES}
)
target_link_libraries(imuSolve
${PROJECT_NAME} #使用头文件的依赖库
${catkin_LIBRARIES}
)
target_link_libraries(eskftest
${PROJECT_NAME} #使用头文件的依赖库
${catkin_LIBRARIES}
)


