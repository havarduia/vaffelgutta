// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from aruco_interfaces:msg/ArucoMarkers.idl
// generated code does not contain a copyright notice

#ifndef ARUCO_INTERFACES__MSG__DETAIL__ARUCO_MARKERS__TRAITS_HPP_
#define ARUCO_INTERFACES__MSG__DETAIL__ARUCO_MARKERS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "aruco_interfaces/msg/detail/aruco_markers__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'poses'
#include "geometry_msgs/msg/detail/pose__traits.hpp"

namespace aruco_interfaces
{

namespace msg
{

inline void to_flow_style_yaml(
  const ArucoMarkers & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: marker_ids
  {
    if (msg.marker_ids.size() == 0) {
      out << "marker_ids: []";
    } else {
      out << "marker_ids: [";
      size_t pending_items = msg.marker_ids.size();
      for (auto item : msg.marker_ids) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: poses
  {
    if (msg.poses.size() == 0) {
      out << "poses: []";
    } else {
      out << "poses: [";
      size_t pending_items = msg.poses.size();
      for (auto item : msg.poses) {
        to_flow_style_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ArucoMarkers & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: marker_ids
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.marker_ids.size() == 0) {
      out << "marker_ids: []\n";
    } else {
      out << "marker_ids:\n";
      for (auto item : msg.marker_ids) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: poses
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.poses.size() == 0) {
      out << "poses: []\n";
    } else {
      out << "poses:\n";
      for (auto item : msg.poses) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "-\n";
        to_block_style_yaml(item, out, indentation + 2);
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ArucoMarkers & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace aruco_interfaces

namespace rosidl_generator_traits
{

[[deprecated("use aruco_interfaces::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const aruco_interfaces::msg::ArucoMarkers & msg,
  std::ostream & out, size_t indentation = 0)
{
  aruco_interfaces::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use aruco_interfaces::msg::to_yaml() instead")]]
inline std::string to_yaml(const aruco_interfaces::msg::ArucoMarkers & msg)
{
  return aruco_interfaces::msg::to_yaml(msg);
}

template<>
inline const char * data_type<aruco_interfaces::msg::ArucoMarkers>()
{
  return "aruco_interfaces::msg::ArucoMarkers";
}

template<>
inline const char * name<aruco_interfaces::msg::ArucoMarkers>()
{
  return "aruco_interfaces/msg/ArucoMarkers";
}

template<>
struct has_fixed_size<aruco_interfaces::msg::ArucoMarkers>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<aruco_interfaces::msg::ArucoMarkers>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<aruco_interfaces::msg::ArucoMarkers>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ARUCO_INTERFACES__MSG__DETAIL__ARUCO_MARKERS__TRAITS_HPP_
