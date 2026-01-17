// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from mav_planning_msgs:msg/PolynomialSegment.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "mav_planning_msgs/msg/polynomial_segment.hpp"


#ifndef MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT__TRAITS_HPP_
#define MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "mav_planning_msgs/msg/detail/polynomial_segment__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"
// Member 'segment_time'
#include "builtin_interfaces/msg/detail/duration__traits.hpp"

namespace mav_planning_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PolynomialSegment & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: num_coeffs
  {
    out << "num_coeffs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_coeffs, out);
    out << ", ";
  }

  // member: segment_time
  {
    out << "segment_time: ";
    to_flow_style_yaml(msg.segment_time, out);
    out << ", ";
  }

  // member: x
  {
    if (msg.x.size() == 0) {
      out << "x: []";
    } else {
      out << "x: [";
      size_t pending_items = msg.x.size();
      for (auto item : msg.x) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: y
  {
    if (msg.y.size() == 0) {
      out << "y: []";
    } else {
      out << "y: [";
      size_t pending_items = msg.y.size();
      for (auto item : msg.y) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: z
  {
    if (msg.z.size() == 0) {
      out << "z: []";
    } else {
      out << "z: [";
      size_t pending_items = msg.z.size();
      for (auto item : msg.z) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: rx
  {
    if (msg.rx.size() == 0) {
      out << "rx: []";
    } else {
      out << "rx: [";
      size_t pending_items = msg.rx.size();
      for (auto item : msg.rx) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: ry
  {
    if (msg.ry.size() == 0) {
      out << "ry: []";
    } else {
      out << "ry: [";
      size_t pending_items = msg.ry.size();
      for (auto item : msg.ry) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: rz
  {
    if (msg.rz.size() == 0) {
      out << "rz: []";
    } else {
      out << "rz: [";
      size_t pending_items = msg.rz.size();
      for (auto item : msg.rz) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
    out << ", ";
  }

  // member: yaw
  {
    if (msg.yaw.size() == 0) {
      out << "yaw: []";
    } else {
      out << "yaw: [";
      size_t pending_items = msg.yaw.size();
      for (auto item : msg.yaw) {
        rosidl_generator_traits::value_to_yaml(item, out);
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
  const PolynomialSegment & msg,
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

  // member: num_coeffs
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "num_coeffs: ";
    rosidl_generator_traits::value_to_yaml(msg.num_coeffs, out);
    out << "\n";
  }

  // member: segment_time
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "segment_time:\n";
    to_block_style_yaml(msg.segment_time, out, indentation + 2);
  }

  // member: x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.x.size() == 0) {
      out << "x: []\n";
    } else {
      out << "x:\n";
      for (auto item : msg.x) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.y.size() == 0) {
      out << "y: []\n";
    } else {
      out << "y:\n";
      for (auto item : msg.y) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: z
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.z.size() == 0) {
      out << "z: []\n";
    } else {
      out << "z:\n";
      for (auto item : msg.z) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: rx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.rx.size() == 0) {
      out << "rx: []\n";
    } else {
      out << "rx:\n";
      for (auto item : msg.rx) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: ry
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.ry.size() == 0) {
      out << "ry: []\n";
    } else {
      out << "ry:\n";
      for (auto item : msg.ry) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: rz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.rz.size() == 0) {
      out << "rz: []\n";
    } else {
      out << "rz:\n";
      for (auto item : msg.rz) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }

  // member: yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.yaw.size() == 0) {
      out << "yaw: []\n";
    } else {
      out << "yaw:\n";
      for (auto item : msg.yaw) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PolynomialSegment & msg, bool use_flow_style = false)
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

}  // namespace mav_planning_msgs

namespace rosidl_generator_traits
{

[[deprecated("use mav_planning_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const mav_planning_msgs::msg::PolynomialSegment & msg,
  std::ostream & out, size_t indentation = 0)
{
  mav_planning_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use mav_planning_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const mav_planning_msgs::msg::PolynomialSegment & msg)
{
  return mav_planning_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<mav_planning_msgs::msg::PolynomialSegment>()
{
  return "mav_planning_msgs::msg::PolynomialSegment";
}

template<>
inline const char * name<mav_planning_msgs::msg::PolynomialSegment>()
{
  return "mav_planning_msgs/msg/PolynomialSegment";
}

template<>
struct has_fixed_size<mav_planning_msgs::msg::PolynomialSegment>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<mav_planning_msgs::msg::PolynomialSegment>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<mav_planning_msgs::msg::PolynomialSegment>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // MAV_PLANNING_MSGS__MSG__DETAIL__POLYNOMIAL_SEGMENT__TRAITS_HPP_
