# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ship_los: 10 messages, 1 services")

set(MSG_I_FLAGS "-Iship_los:/home/bo/dev/REMUS/src/ship_los/msg;-Iship_los:/home/bo/dev/REMUS/devel/share/ship_los/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg;-Ivisualization_msgs:/opt/ros/indigo/share/visualization_msgs/cmake/../msg;-Iactionlib_msgs:/opt/ros/indigo/share/actionlib_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/indigo/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ship_los_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/srv/waypoint.srv" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/src/ship_los/srv/waypoint.srv" ""
)

get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg" "ship_los/WaypointTrackingFeedback:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/pose.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/src/ship_los/msg/pose.msg" ""
)

get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/course.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/src/ship_los/msg/course.msg" ""
)

get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg" ""
)

get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingAction.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingAction.msg" "ship_los/WaypointTrackingFeedback:ship_los/WaypointTrackingActionResult:ship_los/WaypointTrackingActionGoal:actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:ship_los/WaypointTrackingGoal:ship_los/WaypointTrackingResult:std_msgs/Header:ship_los/WaypointTrackingActionFeedback"
)

get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg" "actionlib_msgs/GoalStatus:actionlib_msgs/GoalID:ship_los/WaypointTrackingResult:std_msgs/Header"
)

get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg" "ship_los/WaypointTrackingGoal:actionlib_msgs/GoalID:std_msgs/Header"
)

get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg" ""
)

get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg" ""
)

get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/control.msg" NAME_WE)
add_custom_target(_ship_los_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ship_los" "/home/bo/dev/REMUS/src/ship_los/msg/control.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/course.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)
_generate_msg_cpp(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)

### Generating Services
_generate_srv_cpp(ship_los
  "/home/bo/dev/REMUS/src/ship_los/srv/waypoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
)

### Generating Module File
_generate_module_cpp(ship_los
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ship_los_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ship_los_generate_messages ship_los_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/srv/waypoint.srv" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/pose.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/course.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingAction.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/control.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_cpp _ship_los_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ship_los_gencpp)
add_dependencies(ship_los_gencpp ship_los_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ship_los_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/course.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)
_generate_msg_lisp(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)

### Generating Services
_generate_srv_lisp(ship_los
  "/home/bo/dev/REMUS/src/ship_los/srv/waypoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
)

### Generating Module File
_generate_module_lisp(ship_los
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ship_los_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ship_los_generate_messages ship_los_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/srv/waypoint.srv" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/pose.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/course.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingAction.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/control.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_lisp _ship_los_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ship_los_genlisp)
add_dependencies(ship_los_genlisp ship_los_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ship_los_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/course.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingAction.msg"
  "${MSG_I_FLAGS}"
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalStatus.msg;/opt/ros/indigo/share/actionlib_msgs/cmake/../msg/GoalID.msg;/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg;/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)
_generate_msg_py(ship_los
  "/home/bo/dev/REMUS/src/ship_los/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)

### Generating Services
_generate_srv_py(ship_los
  "/home/bo/dev/REMUS/src/ship_los/srv/waypoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
)

### Generating Module File
_generate_module_py(ship_los
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ship_los_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ship_los_generate_messages ship_los_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/srv/waypoint.srv" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionFeedback.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/pose.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/course.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingFeedback.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingAction.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionResult.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingActionGoal.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingGoal.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/devel/share/ship_los/msg/WaypointTrackingResult.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/ship_los/msg/control.msg" NAME_WE)
add_dependencies(ship_los_generate_messages_py _ship_los_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ship_los_genpy)
add_dependencies(ship_los_genpy ship_los_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ship_los_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ship_los
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(ship_los_generate_messages_cpp std_msgs_generate_messages_cpp)
add_dependencies(ship_los_generate_messages_cpp visualization_msgs_generate_messages_cpp)
add_dependencies(ship_los_generate_messages_cpp actionlib_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ship_los
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(ship_los_generate_messages_lisp std_msgs_generate_messages_lisp)
add_dependencies(ship_los_generate_messages_lisp visualization_msgs_generate_messages_lisp)
add_dependencies(ship_los_generate_messages_lisp actionlib_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ship_los
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(ship_los_generate_messages_py std_msgs_generate_messages_py)
add_dependencies(ship_los_generate_messages_py visualization_msgs_generate_messages_py)
add_dependencies(ship_los_generate_messages_py actionlib_msgs_generate_messages_py)
