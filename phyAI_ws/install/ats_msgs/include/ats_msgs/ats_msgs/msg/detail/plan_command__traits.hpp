// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ats_msgs:msg/PlanCommand.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__TRAITS_HPP_
#define ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ats_msgs/msg/detail/plan_command__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace ats_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const PlanCommand & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: mission_id
  {
    out << "mission_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_id, out);
    out << ", ";
  }

  // member: plan_json
  {
    out << "plan_json: ";
    rosidl_generator_traits::value_to_yaml(msg.plan_json, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const PlanCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: stamp
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "stamp:\n";
    to_block_style_yaml(msg.stamp, out, indentation + 2);
  }

  // member: mission_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mission_id: ";
    rosidl_generator_traits::value_to_yaml(msg.mission_id, out);
    out << "\n";
  }

  // member: plan_json
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "plan_json: ";
    rosidl_generator_traits::value_to_yaml(msg.plan_json, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const PlanCommand & msg, bool use_flow_style = false)
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

}  // namespace ats_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ats_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ats_msgs::msg::PlanCommand & msg,
  std::ostream & out, size_t indentation = 0)
{
  ats_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ats_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ats_msgs::msg::PlanCommand & msg)
{
  return ats_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ats_msgs::msg::PlanCommand>()
{
  return "ats_msgs::msg::PlanCommand";
}

template<>
inline const char * name<ats_msgs::msg::PlanCommand>()
{
  return "ats_msgs/msg/PlanCommand";
}

template<>
struct has_fixed_size<ats_msgs::msg::PlanCommand>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ats_msgs::msg::PlanCommand>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ats_msgs::msg::PlanCommand>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__TRAITS_HPP_
