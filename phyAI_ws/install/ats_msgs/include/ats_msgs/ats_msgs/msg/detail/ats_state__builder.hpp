// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ats_msgs:msg/AtsState.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__ATS_STATE__BUILDER_HPP_
#define ATS_MSGS__MSG__DETAIL__ATS_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ats_msgs/msg/detail/ats_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ats_msgs
{

namespace msg
{

namespace builder
{

class Init_AtsState_events_json
{
public:
  explicit Init_AtsState_events_json(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  ::ats_msgs::msg::AtsState events_json(::ats_msgs::msg::AtsState::_events_json_type arg)
  {
    msg_.events_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_last_violation
{
public:
  explicit Init_AtsState_last_violation(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_events_json last_violation(::ats_msgs::msg::AtsState::_last_violation_type arg)
  {
    msg_.last_violation = std::move(arg);
    return Init_AtsState_events_json(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_max_speed
{
public:
  explicit Init_AtsState_max_speed(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_last_violation max_speed(::ats_msgs::msg::AtsState::_max_speed_type arg)
  {
    msg_.max_speed = std::move(arg);
    return Init_AtsState_last_violation(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_safe_backstop
{
public:
  explicit Init_AtsState_safe_backstop(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_max_speed safe_backstop(::ats_msgs::msg::AtsState::_safe_backstop_type arg)
  {
    msg_.safe_backstop = std::move(arg);
    return Init_AtsState_max_speed(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_roe_ok
{
public:
  explicit Init_AtsState_roe_ok(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_safe_backstop roe_ok(::ats_msgs::msg::AtsState::_roe_ok_type arg)
  {
    msg_.roe_ok = std::move(arg);
    return Init_AtsState_safe_backstop(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_history_json
{
public:
  explicit Init_AtsState_history_json(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_roe_ok history_json(::ats_msgs::msg::AtsState::_history_json_type arg)
  {
    msg_.history_json = std::move(arg);
    return Init_AtsState_roe_ok(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_queue_status
{
public:
  explicit Init_AtsState_queue_status(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_history_json queue_status(::ats_msgs::msg::AtsState::_queue_status_type arg)
  {
    msg_.queue_status = std::move(arg);
    return Init_AtsState_history_json(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_current_index
{
public:
  explicit Init_AtsState_current_index(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_queue_status current_index(::ats_msgs::msg::AtsState::_current_index_type arg)
  {
    msg_.current_index = std::move(arg);
    return Init_AtsState_queue_status(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_plan_json
{
public:
  explicit Init_AtsState_plan_json(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_current_index plan_json(::ats_msgs::msg::AtsState::_plan_json_type arg)
  {
    msg_.plan_json = std::move(arg);
    return Init_AtsState_current_index(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_vision_json
{
public:
  explicit Init_AtsState_vision_json(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_plan_json vision_json(::ats_msgs::msg::AtsState::_vision_json_type arg)
  {
    msg_.vision_json = std::move(arg);
    return Init_AtsState_plan_json(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_lost_sec
{
public:
  explicit Init_AtsState_lost_sec(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_vision_json lost_sec(::ats_msgs::msg::AtsState::_lost_sec_type arg)
  {
    msg_.lost_sec = std::move(arg);
    return Init_AtsState_vision_json(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_primary_id
{
public:
  explicit Init_AtsState_primary_id(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_lost_sec primary_id(::ats_msgs::msg::AtsState::_primary_id_type arg)
  {
    msg_.primary_id = std::move(arg);
    return Init_AtsState_lost_sec(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_battery_voltage
{
public:
  explicit Init_AtsState_battery_voltage(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_primary_id battery_voltage(::ats_msgs::msg::AtsState::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_AtsState_primary_id(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_battery_soc
{
public:
  explicit Init_AtsState_battery_soc(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_battery_voltage battery_soc(::ats_msgs::msg::AtsState::_battery_soc_type arg)
  {
    msg_.battery_soc = std::move(arg);
    return Init_AtsState_battery_voltage(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_vel_wz
{
public:
  explicit Init_AtsState_vel_wz(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_battery_soc vel_wz(::ats_msgs::msg::AtsState::_vel_wz_type arg)
  {
    msg_.vel_wz = std::move(arg);
    return Init_AtsState_battery_soc(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_vel_vy
{
public:
  explicit Init_AtsState_vel_vy(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_vel_wz vel_vy(::ats_msgs::msg::AtsState::_vel_vy_type arg)
  {
    msg_.vel_vy = std::move(arg);
    return Init_AtsState_vel_wz(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_vel_vx
{
public:
  explicit Init_AtsState_vel_vx(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_vel_vy vel_vx(::ats_msgs::msg::AtsState::_vel_vx_type arg)
  {
    msg_.vel_vx = std::move(arg);
    return Init_AtsState_vel_vy(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_pose_yaw
{
public:
  explicit Init_AtsState_pose_yaw(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_vel_vx pose_yaw(::ats_msgs::msg::AtsState::_pose_yaw_type arg)
  {
    msg_.pose_yaw = std::move(arg);
    return Init_AtsState_vel_vx(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_pose_y
{
public:
  explicit Init_AtsState_pose_y(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_pose_yaw pose_y(::ats_msgs::msg::AtsState::_pose_y_type arg)
  {
    msg_.pose_y = std::move(arg);
    return Init_AtsState_pose_yaw(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_pose_x
{
public:
  explicit Init_AtsState_pose_x(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_pose_y pose_x(::ats_msgs::msg::AtsState::_pose_x_type arg)
  {
    msg_.pose_x = std::move(arg);
    return Init_AtsState_pose_y(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_pose_frame
{
public:
  explicit Init_AtsState_pose_frame(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_pose_x pose_frame(::ats_msgs::msg::AtsState::_pose_frame_type arg)
  {
    msg_.pose_frame = std::move(arg);
    return Init_AtsState_pose_x(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_system_state
{
public:
  explicit Init_AtsState_system_state(::ats_msgs::msg::AtsState & msg)
  : msg_(msg)
  {}
  Init_AtsState_pose_frame system_state(::ats_msgs::msg::AtsState::_system_state_type arg)
  {
    msg_.system_state = std::move(arg);
    return Init_AtsState_pose_frame(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

class Init_AtsState_stamp
{
public:
  Init_AtsState_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_AtsState_system_state stamp(::ats_msgs::msg::AtsState::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_AtsState_system_state(msg_);
  }

private:
  ::ats_msgs::msg::AtsState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ats_msgs::msg::AtsState>()
{
  return ats_msgs::msg::builder::Init_AtsState_stamp();
}

}  // namespace ats_msgs

#endif  // ATS_MSGS__MSG__DETAIL__ATS_STATE__BUILDER_HPP_
