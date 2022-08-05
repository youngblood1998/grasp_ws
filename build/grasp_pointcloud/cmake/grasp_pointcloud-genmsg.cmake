# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "grasp_pointcloud: 2 messages, 0 services")

set(MSG_I_FLAGS "-Igrasp_pointcloud:/home/jay/grasp_ws/src/grasp_pointcloud/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(grasp_pointcloud_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg" NAME_WE)
add_custom_target(_grasp_pointcloud_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "grasp_pointcloud" "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg" NAME_WE)
add_custom_target(_grasp_pointcloud_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "grasp_pointcloud" "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grasp_pointcloud
)
_generate_msg_cpp(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grasp_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_cpp(grasp_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grasp_pointcloud
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(grasp_pointcloud_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(grasp_pointcloud_generate_messages grasp_pointcloud_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_cpp _grasp_pointcloud_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_cpp _grasp_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grasp_pointcloud_gencpp)
add_dependencies(grasp_pointcloud_gencpp grasp_pointcloud_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grasp_pointcloud_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grasp_pointcloud
)
_generate_msg_eus(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grasp_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_eus(grasp_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grasp_pointcloud
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(grasp_pointcloud_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(grasp_pointcloud_generate_messages grasp_pointcloud_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_eus _grasp_pointcloud_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_eus _grasp_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grasp_pointcloud_geneus)
add_dependencies(grasp_pointcloud_geneus grasp_pointcloud_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grasp_pointcloud_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grasp_pointcloud
)
_generate_msg_lisp(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grasp_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_lisp(grasp_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grasp_pointcloud
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(grasp_pointcloud_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(grasp_pointcloud_generate_messages grasp_pointcloud_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_lisp _grasp_pointcloud_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_lisp _grasp_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grasp_pointcloud_genlisp)
add_dependencies(grasp_pointcloud_genlisp grasp_pointcloud_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grasp_pointcloud_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grasp_pointcloud
)
_generate_msg_nodejs(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grasp_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_nodejs(grasp_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grasp_pointcloud
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(grasp_pointcloud_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(grasp_pointcloud_generate_messages grasp_pointcloud_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_nodejs _grasp_pointcloud_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_nodejs _grasp_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grasp_pointcloud_gennodejs)
add_dependencies(grasp_pointcloud_gennodejs grasp_pointcloud_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grasp_pointcloud_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grasp_pointcloud
)
_generate_msg_py(grasp_pointcloud
  "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grasp_pointcloud
)

### Generating Services

### Generating Module File
_generate_module_py(grasp_pointcloud
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grasp_pointcloud
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(grasp_pointcloud_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(grasp_pointcloud_generate_messages grasp_pointcloud_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/PointBoundingBox.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_py _grasp_pointcloud_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jay/grasp_ws/src/grasp_pointcloud/msg/GraspParams.msg" NAME_WE)
add_dependencies(grasp_pointcloud_generate_messages_py _grasp_pointcloud_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(grasp_pointcloud_genpy)
add_dependencies(grasp_pointcloud_genpy grasp_pointcloud_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS grasp_pointcloud_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grasp_pointcloud)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/grasp_pointcloud
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(grasp_pointcloud_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grasp_pointcloud)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/grasp_pointcloud
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(grasp_pointcloud_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grasp_pointcloud)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/grasp_pointcloud
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(grasp_pointcloud_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grasp_pointcloud)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/grasp_pointcloud
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(grasp_pointcloud_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grasp_pointcloud)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grasp_pointcloud\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/grasp_pointcloud
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(grasp_pointcloud_generate_messages_py std_msgs_generate_messages_py)
endif()
