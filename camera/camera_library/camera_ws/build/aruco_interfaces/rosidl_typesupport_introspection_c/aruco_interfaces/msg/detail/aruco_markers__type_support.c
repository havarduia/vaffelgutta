// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from aruco_interfaces:msg/ArucoMarkers.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "aruco_interfaces/msg/detail/aruco_markers__rosidl_typesupport_introspection_c.h"
#include "aruco_interfaces/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "aruco_interfaces/msg/detail/aruco_markers__functions.h"
#include "aruco_interfaces/msg/detail/aruco_markers__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `marker_ids`
#include "rosidl_runtime_c/primitives_sequence_functions.h"
// Member `poses`
#include "geometry_msgs/msg/pose.h"
// Member `poses`
#include "geometry_msgs/msg/detail/pose__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  aruco_interfaces__msg__ArucoMarkers__init(message_memory);
}

void aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_fini_function(void * message_memory)
{
  aruco_interfaces__msg__ArucoMarkers__fini(message_memory);
}

size_t aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__size_function__ArucoMarkers__marker_ids(
  const void * untyped_member)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return member->size;
}

const void * aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_const_function__ArucoMarkers__marker_ids(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__int64__Sequence * member =
    (const rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void * aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_function__ArucoMarkers__marker_ids(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  return &member->data[index];
}

void aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__fetch_function__ArucoMarkers__marker_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const int64_t * item =
    ((const int64_t *)
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_const_function__ArucoMarkers__marker_ids(untyped_member, index));
  int64_t * value =
    (int64_t *)(untyped_value);
  *value = *item;
}

void aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__assign_function__ArucoMarkers__marker_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  int64_t * item =
    ((int64_t *)
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_function__ArucoMarkers__marker_ids(untyped_member, index));
  const int64_t * value =
    (const int64_t *)(untyped_value);
  *item = *value;
}

bool aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__resize_function__ArucoMarkers__marker_ids(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__int64__Sequence * member =
    (rosidl_runtime_c__int64__Sequence *)(untyped_member);
  rosidl_runtime_c__int64__Sequence__fini(member);
  return rosidl_runtime_c__int64__Sequence__init(member, size);
}

size_t aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__size_function__ArucoMarkers__poses(
  const void * untyped_member)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return member->size;
}

const void * aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_const_function__ArucoMarkers__poses(
  const void * untyped_member, size_t index)
{
  const geometry_msgs__msg__Pose__Sequence * member =
    (const geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void * aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_function__ArucoMarkers__poses(
  void * untyped_member, size_t index)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  return &member->data[index];
}

void aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__fetch_function__ArucoMarkers__poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const geometry_msgs__msg__Pose * item =
    ((const geometry_msgs__msg__Pose *)
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_const_function__ArucoMarkers__poses(untyped_member, index));
  geometry_msgs__msg__Pose * value =
    (geometry_msgs__msg__Pose *)(untyped_value);
  *value = *item;
}

void aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__assign_function__ArucoMarkers__poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  geometry_msgs__msg__Pose * item =
    ((geometry_msgs__msg__Pose *)
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_function__ArucoMarkers__poses(untyped_member, index));
  const geometry_msgs__msg__Pose * value =
    (const geometry_msgs__msg__Pose *)(untyped_value);
  *item = *value;
}

bool aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__resize_function__ArucoMarkers__poses(
  void * untyped_member, size_t size)
{
  geometry_msgs__msg__Pose__Sequence * member =
    (geometry_msgs__msg__Pose__Sequence *)(untyped_member);
  geometry_msgs__msg__Pose__Sequence__fini(member);
  return geometry_msgs__msg__Pose__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_member_array[3] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces__msg__ArucoMarkers, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "marker_ids",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces__msg__ArucoMarkers, marker_ids),  // bytes offset in struct
    NULL,  // default value
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__size_function__ArucoMarkers__marker_ids,  // size() function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_const_function__ArucoMarkers__marker_ids,  // get_const(index) function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_function__ArucoMarkers__marker_ids,  // get(index) function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__fetch_function__ArucoMarkers__marker_ids,  // fetch(index, &value) function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__assign_function__ArucoMarkers__marker_ids,  // assign(index, value) function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__resize_function__ArucoMarkers__marker_ids  // resize(index) function pointer
  },
  {
    "poses",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces__msg__ArucoMarkers, poses),  // bytes offset in struct
    NULL,  // default value
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__size_function__ArucoMarkers__poses,  // size() function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_const_function__ArucoMarkers__poses,  // get_const(index) function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__get_function__ArucoMarkers__poses,  // get(index) function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__fetch_function__ArucoMarkers__poses,  // fetch(index, &value) function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__assign_function__ArucoMarkers__poses,  // assign(index, value) function pointer
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__resize_function__ArucoMarkers__poses  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_members = {
  "aruco_interfaces__msg",  // message namespace
  "ArucoMarkers",  // message name
  3,  // number of fields
  sizeof(aruco_interfaces__msg__ArucoMarkers),
  aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_member_array,  // message members
  aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_init_function,  // function to initialize message memory (memory has to be allocated)
  aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_type_support_handle = {
  0,
  &aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_aruco_interfaces
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, aruco_interfaces, msg, ArucoMarkers)() {
  aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_member_array[2].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, geometry_msgs, msg, Pose)();
  if (!aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_type_support_handle.typesupport_identifier) {
    aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &aruco_interfaces__msg__ArucoMarkers__rosidl_typesupport_introspection_c__ArucoMarkers_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
