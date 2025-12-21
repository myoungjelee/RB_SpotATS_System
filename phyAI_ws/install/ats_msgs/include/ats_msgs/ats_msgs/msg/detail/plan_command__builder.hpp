// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ats_msgs:msg/PlanCommand.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__BUILDER_HPP_
#define ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ats_msgs/msg/detail/plan_command__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ats_msgs
{

namespace msg
{

namespace builder
{

class Init_PlanCommand_plan_json
{
public:
  explicit Init_PlanCommand_plan_json(::ats_msgs::msg::PlanCommand & msg)
  : msg_(msg)
  {}
  ::ats_msgs::msg::PlanCommand plan_json(::ats_msgs::msg::PlanCommand::_plan_json_type arg)
  {
    msg_.plan_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ats_msgs::msg::PlanCommand msg_;
};

class Init_PlanCommand_mission_id
{
public:
  explicit Init_PlanCommand_mission_id(::ats_msgs::msg::PlanCommand & msg)
  : msg_(msg)
  {}
  Init_PlanCommand_plan_json mission_id(::ats_msgs::msg::PlanCommand::_mission_id_type arg)
  {
    msg_.mission_id = std::move(arg);
    return Init_PlanCommand_plan_json(msg_);
  }

private:
  ::ats_msgs::msg::PlanCommand msg_;
};

class Init_PlanCommand_stamp
{
public:
  Init_PlanCommand_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_PlanCommand_mission_id stamp(::ats_msgs::msg::PlanCommand::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_PlanCommand_mission_id(msg_);
  }

private:
  ::ats_msgs::msg::PlanCommand msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ats_msgs::msg::PlanCommand>()
{
  return ats_msgs::msg::builder::Init_PlanCommand_stamp();
}

}  // namespace ats_msgs

#endif  // ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__BUILDER_HPP_
