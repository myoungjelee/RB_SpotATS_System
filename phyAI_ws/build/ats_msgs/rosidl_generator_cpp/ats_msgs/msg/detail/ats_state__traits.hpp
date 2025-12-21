// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ats_msgs:msg/AtsState.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__ATS_STATE__TRAITS_HPP_
#define ATS_MSGS__MSG__DETAIL__ATS_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ats_msgs/msg/detail/ats_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__traits.hpp"

namespace ats_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const AtsState & msg,
  std::ostream & out)
{
  out << "{";
  // member: stamp
  {
    out << "stamp: ";
    to_flow_style_yaml(msg.stamp, out);
    out << ", ";
  }

  // member: system_state
  {
    out << "system_state: ";
    rosidl_generator_traits::value_to_yaml(msg.system_state, out);
    out << ", ";
  }

  // member: pose_frame
  {
    out << "pose_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_frame, out);
    out << ", ";
  }

  // member: pose_x
  {
    out << "pose_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_x, out);
    out << ", ";
  }

  // member: pose_y
  {
    out << "pose_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_y, out);
    out << ", ";
  }

  // member: pose_yaw
  {
    out << "pose_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_yaw, out);
    out << ", ";
  }

  // member: vel_vx
  {
    out << "vel_vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_vx, out);
    out << ", ";
  }

  // member: vel_vy
  {
    out << "vel_vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_vy, out);
    out << ", ";
  }

  // member: vel_wz
  {
    out << "vel_wz: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_wz, out);
    out << ", ";
  }

  // member: battery_soc
  {
    out << "battery_soc: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_soc, out);
    out << ", ";
  }

  // member: battery_voltage
  {
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << ", ";
  }

  // member: primary_id
  {
    out << "primary_id: ";
    rosidl_generator_traits::value_to_yaml(msg.primary_id, out);
    out << ", ";
  }

  // member: lost_sec
  {
    out << "lost_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.lost_sec, out);
    out << ", ";
  }

  // member: vision_json
  {
    out << "vision_json: ";
    rosidl_generator_traits::value_to_yaml(msg.vision_json, out);
    out << ", ";
  }

  // member: plan_json
  {
    out << "plan_json: ";
    rosidl_generator_traits::value_to_yaml(msg.plan_json, out);
    out << ", ";
  }

  // member: current_index
  {
    out << "current_index: ";
    rosidl_generator_traits::value_to_yaml(msg.current_index, out);
    out << ", ";
  }

  // member: queue_status
  {
    out << "queue_status: ";
    rosidl_generator_traits::value_to_yaml(msg.queue_status, out);
    out << ", ";
  }

  // member: history_json
  {
    out << "history_json: ";
    rosidl_generator_traits::value_to_yaml(msg.history_json, out);
    out << ", ";
  }

  // member: roe_ok
  {
    out << "roe_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.roe_ok, out);
    out << ", ";
  }

  // member: safe_backstop
  {
    out << "safe_backstop: ";
    rosidl_generator_traits::value_to_yaml(msg.safe_backstop, out);
    out << ", ";
  }

  // member: max_speed
  {
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << ", ";
  }

  // member: last_violation
  {
    out << "last_violation: ";
    rosidl_generator_traits::value_to_yaml(msg.last_violation, out);
    out << ", ";
  }

  // member: events_json
  {
    out << "events_json: ";
    rosidl_generator_traits::value_to_yaml(msg.events_json, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const AtsState & msg,
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

  // member: system_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "system_state: ";
    rosidl_generator_traits::value_to_yaml(msg.system_state, out);
    out << "\n";
  }

  // member: pose_frame
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_frame: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_frame, out);
    out << "\n";
  }

  // member: pose_x
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_x: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_x, out);
    out << "\n";
  }

  // member: pose_y
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_y: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_y, out);
    out << "\n";
  }

  // member: pose_yaw
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pose_yaw: ";
    rosidl_generator_traits::value_to_yaml(msg.pose_yaw, out);
    out << "\n";
  }

  // member: vel_vx
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_vx: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_vx, out);
    out << "\n";
  }

  // member: vel_vy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_vy: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_vy, out);
    out << "\n";
  }

  // member: vel_wz
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vel_wz: ";
    rosidl_generator_traits::value_to_yaml(msg.vel_wz, out);
    out << "\n";
  }

  // member: battery_soc
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_soc: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_soc, out);
    out << "\n";
  }

  // member: battery_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "battery_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.battery_voltage, out);
    out << "\n";
  }

  // member: primary_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "primary_id: ";
    rosidl_generator_traits::value_to_yaml(msg.primary_id, out);
    out << "\n";
  }

  // member: lost_sec
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "lost_sec: ";
    rosidl_generator_traits::value_to_yaml(msg.lost_sec, out);
    out << "\n";
  }

  // member: vision_json
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "vision_json: ";
    rosidl_generator_traits::value_to_yaml(msg.vision_json, out);
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

  // member: current_index
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current_index: ";
    rosidl_generator_traits::value_to_yaml(msg.current_index, out);
    out << "\n";
  }

  // member: queue_status
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "queue_status: ";
    rosidl_generator_traits::value_to_yaml(msg.queue_status, out);
    out << "\n";
  }

  // member: history_json
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "history_json: ";
    rosidl_generator_traits::value_to_yaml(msg.history_json, out);
    out << "\n";
  }

  // member: roe_ok
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "roe_ok: ";
    rosidl_generator_traits::value_to_yaml(msg.roe_ok, out);
    out << "\n";
  }

  // member: safe_backstop
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "safe_backstop: ";
    rosidl_generator_traits::value_to_yaml(msg.safe_backstop, out);
    out << "\n";
  }

  // member: max_speed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "max_speed: ";
    rosidl_generator_traits::value_to_yaml(msg.max_speed, out);
    out << "\n";
  }

  // member: last_violation
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "last_violation: ";
    rosidl_generator_traits::value_to_yaml(msg.last_violation, out);
    out << "\n";
  }

  // member: events_json
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "events_json: ";
    rosidl_generator_traits::value_to_yaml(msg.events_json, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const AtsState & msg, bool use_flow_style = false)
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
  const ats_msgs::msg::AtsState & msg,
  std::ostream & out, size_t indentation = 0)
{
  ats_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ats_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ats_msgs::msg::AtsState & msg)
{
  return ats_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ats_msgs::msg::AtsState>()
{
  return "ats_msgs::msg::AtsState";
}

template<>
inline const char * name<ats_msgs::msg::AtsState>()
{
  return "ats_msgs/msg/AtsState";
}

template<>
struct has_fixed_size<ats_msgs::msg::AtsState>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<ats_msgs::msg::AtsState>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<ats_msgs::msg::AtsState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // ATS_MSGS__MSG__DETAIL__ATS_STATE__TRAITS_HPP_
