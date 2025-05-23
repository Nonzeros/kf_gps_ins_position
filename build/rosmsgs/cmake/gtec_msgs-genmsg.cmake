# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "gtec_msgs: 16 messages, 0 services")

set(MSG_I_FLAGS "-Igtec_msgs:/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(gtec_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg" ""
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg" ""
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg" ""
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg" ""
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg" ""
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg" ""
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg" ""
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg" ""
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg" "gtec_msgs/ESP32S2FTMFrame"
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg" "gtec_msgs/ESP32S2FTMFrame"
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg" "geometry_msgs/Point:std_msgs/Header"
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg" NAME_WE)
add_custom_target(_gtec_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "gtec_msgs" "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg" "std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_cpp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(gtec_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(gtec_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(gtec_msgs_generate_messages gtec_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_cpp _gtec_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gtec_msgs_gencpp)
add_dependencies(gtec_msgs_gencpp gtec_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gtec_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)
_generate_msg_eus(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(gtec_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(gtec_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(gtec_msgs_generate_messages gtec_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_eus _gtec_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gtec_msgs_geneus)
add_dependencies(gtec_msgs_geneus gtec_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gtec_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)
_generate_msg_lisp(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(gtec_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(gtec_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(gtec_msgs_generate_messages gtec_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_lisp _gtec_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gtec_msgs_genlisp)
add_dependencies(gtec_msgs_genlisp gtec_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gtec_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)
_generate_msg_nodejs(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(gtec_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(gtec_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(gtec_msgs_generate_messages gtec_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_nodejs _gtec_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gtec_msgs_gennodejs)
add_dependencies(gtec_msgs_gennodejs gtec_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gtec_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg"
  "${MSG_I_FLAGS}"
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)
_generate_msg_py(gtec_msgs
  "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(gtec_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(gtec_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(gtec_msgs_generate_messages gtec_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/GenericRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/Ranging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/UWBRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DWRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RangingDiff.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/PozyxRangingWithCir.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMFrame.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRanging.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ESP32S2FTMRangingExtra.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarCube.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeAzimuth.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarRangeDoppler.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/RadarFusedPointStamped.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/DoorCounterEvent.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ouyy/MyRosGuideCar/src/rosmsgs/msg-ros1/ZoneOccupancy.msg" NAME_WE)
add_dependencies(gtec_msgs_generate_messages_py _gtec_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(gtec_msgs_genpy)
add_dependencies(gtec_msgs_genpy gtec_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS gtec_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/gtec_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(gtec_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(gtec_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/gtec_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(gtec_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(gtec_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/gtec_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(gtec_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(gtec_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/gtec_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(gtec_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(gtec_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/gtec_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(gtec_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(gtec_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
