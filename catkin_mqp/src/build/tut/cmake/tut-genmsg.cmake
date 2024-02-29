# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "tut: 1 messages, 1 services")

set(MSG_I_FLAGS "-Itut:/home/vboxuser/catkin_mqp/src/tut/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(tut_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg" NAME_WE)
add_custom_target(_tut_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tut" "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg" ""
)

get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_tut_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "tut" "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(tut
  "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tut
)

### Generating Services
_generate_srv_cpp(tut
  "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tut
)

### Generating Module File
_generate_module_cpp(tut
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tut
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(tut_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(tut_generate_messages tut_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg" NAME_WE)
add_dependencies(tut_generate_messages_cpp _tut_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(tut_generate_messages_cpp _tut_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tut_gencpp)
add_dependencies(tut_gencpp tut_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tut_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(tut
  "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tut
)

### Generating Services
_generate_srv_eus(tut
  "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tut
)

### Generating Module File
_generate_module_eus(tut
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tut
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(tut_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(tut_generate_messages tut_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg" NAME_WE)
add_dependencies(tut_generate_messages_eus _tut_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(tut_generate_messages_eus _tut_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tut_geneus)
add_dependencies(tut_geneus tut_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tut_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(tut
  "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tut
)

### Generating Services
_generate_srv_lisp(tut
  "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tut
)

### Generating Module File
_generate_module_lisp(tut
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tut
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(tut_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(tut_generate_messages tut_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg" NAME_WE)
add_dependencies(tut_generate_messages_lisp _tut_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(tut_generate_messages_lisp _tut_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tut_genlisp)
add_dependencies(tut_genlisp tut_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tut_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(tut
  "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tut
)

### Generating Services
_generate_srv_nodejs(tut
  "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tut
)

### Generating Module File
_generate_module_nodejs(tut
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tut
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(tut_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(tut_generate_messages tut_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg" NAME_WE)
add_dependencies(tut_generate_messages_nodejs _tut_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(tut_generate_messages_nodejs _tut_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tut_gennodejs)
add_dependencies(tut_gennodejs tut_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tut_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(tut
  "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tut
)

### Generating Services
_generate_srv_py(tut
  "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tut
)

### Generating Module File
_generate_module_py(tut
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tut
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(tut_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(tut_generate_messages tut_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/msg/Num.msg" NAME_WE)
add_dependencies(tut_generate_messages_py _tut_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/vboxuser/catkin_mqp/src/tut/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(tut_generate_messages_py _tut_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(tut_genpy)
add_dependencies(tut_genpy tut_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS tut_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tut)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/tut
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(tut_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tut)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/tut
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(tut_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tut)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/tut
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(tut_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tut)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/tut
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(tut_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tut)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tut\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/tut
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(tut_generate_messages_py std_msgs_generate_messages_py)
endif()
