// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from aruco_interfaces:msg/ArucoMarkers.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "aruco_interfaces/msg/detail/aruco_markers__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace aruco_interfaces
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void ArucoMarkers_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) aruco_interfaces::msg::ArucoMarkers(_init);
}

void ArucoMarkers_fini_function(void * message_memory)
{
  auto typed_message = static_cast<aruco_interfaces::msg::ArucoMarkers *>(message_memory);
  typed_message->~ArucoMarkers();
}

size_t size_function__ArucoMarkers__marker_ids(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ArucoMarkers__marker_ids(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void * get_function__ArucoMarkers__marker_ids(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  return &member[index];
}

void fetch_function__ArucoMarkers__marker_ids(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const int64_t *>(
    get_const_function__ArucoMarkers__marker_ids(untyped_member, index));
  auto & value = *reinterpret_cast<int64_t *>(untyped_value);
  value = item;
}

void assign_function__ArucoMarkers__marker_ids(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<int64_t *>(
    get_function__ArucoMarkers__marker_ids(untyped_member, index));
  const auto & value = *reinterpret_cast<const int64_t *>(untyped_value);
  item = value;
}

void resize_function__ArucoMarkers__marker_ids(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<int64_t> *>(untyped_member);
  member->resize(size);
}

size_t size_function__ArucoMarkers__poses(const void * untyped_member)
{
  const auto * member = reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return member->size();
}

const void * get_const_function__ArucoMarkers__poses(const void * untyped_member, size_t index)
{
  const auto & member =
    *reinterpret_cast<const std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void * get_function__ArucoMarkers__poses(void * untyped_member, size_t index)
{
  auto & member =
    *reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  return &member[index];
}

void fetch_function__ArucoMarkers__poses(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const auto & item = *reinterpret_cast<const geometry_msgs::msg::Pose *>(
    get_const_function__ArucoMarkers__poses(untyped_member, index));
  auto & value = *reinterpret_cast<geometry_msgs::msg::Pose *>(untyped_value);
  value = item;
}

void assign_function__ArucoMarkers__poses(
  void * untyped_member, size_t index, const void * untyped_value)
{
  auto & item = *reinterpret_cast<geometry_msgs::msg::Pose *>(
    get_function__ArucoMarkers__poses(untyped_member, index));
  const auto & value = *reinterpret_cast<const geometry_msgs::msg::Pose *>(untyped_value);
  item = value;
}

void resize_function__ArucoMarkers__poses(void * untyped_member, size_t size)
{
  auto * member =
    reinterpret_cast<std::vector<geometry_msgs::msg::Pose> *>(untyped_member);
  member->resize(size);
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember ArucoMarkers_message_member_array[3] = {
  {
    "header",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<std_msgs::msg::Header>(),  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces::msg::ArucoMarkers, header),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "marker_ids",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT64,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces::msg::ArucoMarkers, marker_ids),  // bytes offset in struct
    nullptr,  // default value
    size_function__ArucoMarkers__marker_ids,  // size() function pointer
    get_const_function__ArucoMarkers__marker_ids,  // get_const(index) function pointer
    get_function__ArucoMarkers__marker_ids,  // get(index) function pointer
    fetch_function__ArucoMarkers__marker_ids,  // fetch(index, &value) function pointer
    assign_function__ArucoMarkers__marker_ids,  // assign(index, value) function pointer
    resize_function__ArucoMarkers__marker_ids  // resize(index) function pointer
  },
  {
    "poses",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    ::rosidl_typesupport_introspection_cpp::get_message_type_support_handle<geometry_msgs::msg::Pose>(),  // members of sub message
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(aruco_interfaces::msg::ArucoMarkers, poses),  // bytes offset in struct
    nullptr,  // default value
    size_function__ArucoMarkers__poses,  // size() function pointer
    get_const_function__ArucoMarkers__poses,  // get_const(index) function pointer
    get_function__ArucoMarkers__poses,  // get(index) function pointer
    fetch_function__ArucoMarkers__poses,  // fetch(index, &value) function pointer
    assign_function__ArucoMarkers__poses,  // assign(index, value) function pointer
    resize_function__ArucoMarkers__poses  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers ArucoMarkers_message_members = {
  "aruco_interfaces::msg",  // message namespace
  "ArucoMarkers",  // message name
  3,  // number of fields
  sizeof(aruco_interfaces::msg::ArucoMarkers),
  ArucoMarkers_message_member_array,  // message members
  ArucoMarkers_init_function,  // function to initialize message memory (memory has to be allocated)
  ArucoMarkers_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t ArucoMarkers_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &ArucoMarkers_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace aruco_interfaces


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<aruco_interfaces::msg::ArucoMarkers>()
{
  return &::aruco_interfaces::msg::rosidl_typesupport_introspection_cpp::ArucoMarkers_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, aruco_interfaces, msg, ArucoMarkers)() {
  return &::aruco_interfaces::msg::rosidl_typesupport_introspection_cpp::ArucoMarkers_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
