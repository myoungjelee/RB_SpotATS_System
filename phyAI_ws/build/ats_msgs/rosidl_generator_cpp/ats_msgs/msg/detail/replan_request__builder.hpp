// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ats_msgs:msg/ReplanRequest.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__REPLAN_REQUEST__BUILDER_HPP_
#define ATS_MSGS__MSG__DETAIL__REPLAN_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ats_msgs/msg/detail/replan_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ats_msgs
{

namespace msg
{

namespace builder
{

class Init_ReplanRequest_context_json
{
public:
  explicit Init_ReplanRequest_context_json(::ats_msgs::msg::ReplanRequest & msg)
  : msg_(msg)
  {}
  ::ats_msgs::msg::ReplanRequest context_json(::ats_msgs::msg::ReplanRequest::_context_json_type arg)
  {
    msg_.context_json = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ats_msgs::msg::ReplanRequest msg_;
};

class Init_ReplanRequest_reason
{
public:
  explicit Init_ReplanRequest_reason(::ats_msgs::msg::ReplanRequest & msg)
  : msg_(msg)
  {}
  Init_ReplanRequest_context_json reason(::ats_msgs::msg::ReplanRequest::_reason_type arg)
  {
    msg_.reason = std::move(arg);
    return Init_ReplanRequest_context_json(msg_);
  }

private:
  ::ats_msgs::msg::ReplanRequest msg_;
};

class Init_ReplanRequest_mission_id
{
public:
  explicit Init_ReplanRequest_mission_id(::ats_msgs::msg::ReplanRequest & msg)
  : msg_(msg)
  {}
  Init_ReplanRequest_reason mission_id(::ats_msgs::msg::ReplanRequest::_mission_id_type arg)
  {
    msg_.mission_id = std::move(arg);
    return Init_ReplanRequest_reason(msg_);
  }

private:
  ::ats_msgs::msg::ReplanRequest msg_;
};

class Init_ReplanRequest_stamp
{
public:
  Init_ReplanRequest_stamp()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ReplanRequest_mission_id stamp(::ats_msgs::msg::ReplanRequest::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return Init_ReplanRequest_mission_id(msg_);
  }

private:
  ::ats_msgs::msg::ReplanRequest msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ats_msgs::msg::ReplanRequest>()
{
  return ats_msgs::msg::builder::Init_ReplanRequest_stamp();
}

}  // namespace ats_msgs

#endif  // ATS_MSGS__MSG__DETAIL__REPLAN_REQUEST__BUILDER_HPP_
