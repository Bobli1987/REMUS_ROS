# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "imm: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iimm:/home/bo/dev/REMUS/src/imm/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(imm_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/bo/dev/REMUS/src/imm/msg/pose.msg" NAME_WE)
add_custom_target(_imm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "imm" "/home/bo/dev/REMUS/src/imm/msg/pose.msg" ""
)

get_filename_component(_filename "/home/bo/dev/REMUS/src/imm/msg/control.msg" NAME_WE)
add_custom_target(_imm_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "imm" "/home/bo/dev/REMUS/src/imm/msg/control.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(imm
  "/home/bo/dev/REMUS/src/imm/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imm
)
_generate_msg_cpp(imm
  "/home/bo/dev/REMUS/src/imm/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imm
)

### Generating Services

### Generating Module File
_generate_module_cpp(imm
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imm
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(imm_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(imm_generate_messages imm_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bo/dev/REMUS/src/imm/msg/pose.msg" NAME_WE)
add_dependencies(imm_generate_messages_cpp _imm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/imm/msg/control.msg" NAME_WE)
add_dependencies(imm_generate_messages_cpp _imm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(imm_gencpp)
add_dependencies(imm_gencpp imm_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imm_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(imm
  "/home/bo/dev/REMUS/src/imm/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imm
)
_generate_msg_lisp(imm
  "/home/bo/dev/REMUS/src/imm/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imm
)

### Generating Services

### Generating Module File
_generate_module_lisp(imm
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imm
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(imm_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(imm_generate_messages imm_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bo/dev/REMUS/src/imm/msg/pose.msg" NAME_WE)
add_dependencies(imm_generate_messages_lisp _imm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/imm/msg/control.msg" NAME_WE)
add_dependencies(imm_generate_messages_lisp _imm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(imm_genlisp)
add_dependencies(imm_genlisp imm_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imm_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(imm
  "/home/bo/dev/REMUS/src/imm/msg/pose.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imm
)
_generate_msg_py(imm
  "/home/bo/dev/REMUS/src/imm/msg/control.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imm
)

### Generating Services

### Generating Module File
_generate_module_py(imm
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imm
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(imm_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(imm_generate_messages imm_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/bo/dev/REMUS/src/imm/msg/pose.msg" NAME_WE)
add_dependencies(imm_generate_messages_py _imm_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/bo/dev/REMUS/src/imm/msg/control.msg" NAME_WE)
add_dependencies(imm_generate_messages_py _imm_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(imm_genpy)
add_dependencies(imm_genpy imm_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS imm_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/imm
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(imm_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imm)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/imm
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(imm_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imm)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imm\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/imm
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(imm_generate_messages_py std_msgs_generate_messages_py)
