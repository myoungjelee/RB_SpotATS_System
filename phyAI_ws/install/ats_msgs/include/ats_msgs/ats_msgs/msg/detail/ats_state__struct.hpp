// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ats_msgs:msg/AtsState.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__ATS_STATE__STRUCT_HPP_
#define ATS_MSGS__MSG__DETAIL__ATS_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ats_msgs__msg__AtsState __attribute__((deprecated))
#else
# define DEPRECATED__ats_msgs__msg__AtsState __declspec(deprecated)
#endif

namespace ats_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct AtsState_
{
  using Type = AtsState_<ContainerAllocator>;

  explicit AtsState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->system_state = "";
      this->pose_frame = "";
      this->pose_x = 0.0;
      this->pose_y = 0.0;
      this->pose_yaw = 0.0;
      this->vel_vx = 0.0;
      this->vel_vy = 0.0;
      this->vel_wz = 0.0;
      this->battery_soc = 0.0;
      this->battery_voltage = 0.0;
      this->primary_id = 0l;
      this->lost_sec = 0.0;
      this->vision_json = "";
      this->plan_json = "";
      this->current_index = 0l;
      this->queue_status = "";
      this->history_json = "";
      this->roe_ok = false;
      this->safe_backstop = false;
      this->max_speed = 0.0;
      this->last_violation = "";
      this->events_json = "";
    }
  }

  explicit AtsState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    system_state(_alloc),
    pose_frame(_alloc),
    vision_json(_alloc),
    plan_json(_alloc),
    queue_status(_alloc),
    history_json(_alloc),
    last_violation(_alloc),
    events_json(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->system_state = "";
      this->pose_frame = "";
      this->pose_x = 0.0;
      this->pose_y = 0.0;
      this->pose_yaw = 0.0;
      this->vel_vx = 0.0;
      this->vel_vy = 0.0;
      this->vel_wz = 0.0;
      this->battery_soc = 0.0;
      this->battery_voltage = 0.0;
      this->primary_id = 0l;
      this->lost_sec = 0.0;
      this->vision_json = "";
      this->plan_json = "";
      this->current_index = 0l;
      this->queue_status = "";
      this->history_json = "";
      this->roe_ok = false;
      this->safe_backstop = false;
      this->max_speed = 0.0;
      this->last_violation = "";
      this->events_json = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _system_state_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _system_state_type system_state;
  using _pose_frame_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _pose_frame_type pose_frame;
  using _pose_x_type =
    double;
  _pose_x_type pose_x;
  using _pose_y_type =
    double;
  _pose_y_type pose_y;
  using _pose_yaw_type =
    double;
  _pose_yaw_type pose_yaw;
  using _vel_vx_type =
    double;
  _vel_vx_type vel_vx;
  using _vel_vy_type =
    double;
  _vel_vy_type vel_vy;
  using _vel_wz_type =
    double;
  _vel_wz_type vel_wz;
  using _battery_soc_type =
    double;
  _battery_soc_type battery_soc;
  using _battery_voltage_type =
    double;
  _battery_voltage_type battery_voltage;
  using _primary_id_type =
    int32_t;
  _primary_id_type primary_id;
  using _lost_sec_type =
    double;
  _lost_sec_type lost_sec;
  using _vision_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _vision_json_type vision_json;
  using _plan_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _plan_json_type plan_json;
  using _current_index_type =
    int32_t;
  _current_index_type current_index;
  using _queue_status_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _queue_status_type queue_status;
  using _history_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _history_json_type history_json;
  using _roe_ok_type =
    bool;
  _roe_ok_type roe_ok;
  using _safe_backstop_type =
    bool;
  _safe_backstop_type safe_backstop;
  using _max_speed_type =
    double;
  _max_speed_type max_speed;
  using _last_violation_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _last_violation_type last_violation;
  using _events_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _events_json_type events_json;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__system_state(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->system_state = _arg;
    return *this;
  }
  Type & set__pose_frame(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->pose_frame = _arg;
    return *this;
  }
  Type & set__pose_x(
    const double & _arg)
  {
    this->pose_x = _arg;
    return *this;
  }
  Type & set__pose_y(
    const double & _arg)
  {
    this->pose_y = _arg;
    return *this;
  }
  Type & set__pose_yaw(
    const double & _arg)
  {
    this->pose_yaw = _arg;
    return *this;
  }
  Type & set__vel_vx(
    const double & _arg)
  {
    this->vel_vx = _arg;
    return *this;
  }
  Type & set__vel_vy(
    const double & _arg)
  {
    this->vel_vy = _arg;
    return *this;
  }
  Type & set__vel_wz(
    const double & _arg)
  {
    this->vel_wz = _arg;
    return *this;
  }
  Type & set__battery_soc(
    const double & _arg)
  {
    this->battery_soc = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const double & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__primary_id(
    const int32_t & _arg)
  {
    this->primary_id = _arg;
    return *this;
  }
  Type & set__lost_sec(
    const double & _arg)
  {
    this->lost_sec = _arg;
    return *this;
  }
  Type & set__vision_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->vision_json = _arg;
    return *this;
  }
  Type & set__plan_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->plan_json = _arg;
    return *this;
  }
  Type & set__current_index(
    const int32_t & _arg)
  {
    this->current_index = _arg;
    return *this;
  }
  Type & set__queue_status(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->queue_status = _arg;
    return *this;
  }
  Type & set__history_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->history_json = _arg;
    return *this;
  }
  Type & set__roe_ok(
    const bool & _arg)
  {
    this->roe_ok = _arg;
    return *this;
  }
  Type & set__safe_backstop(
    const bool & _arg)
  {
    this->safe_backstop = _arg;
    return *this;
  }
  Type & set__max_speed(
    const double & _arg)
  {
    this->max_speed = _arg;
    return *this;
  }
  Type & set__last_violation(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->last_violation = _arg;
    return *this;
  }
  Type & set__events_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->events_json = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ats_msgs::msg::AtsState_<ContainerAllocator> *;
  using ConstRawPtr =
    const ats_msgs::msg::AtsState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ats_msgs::msg::AtsState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ats_msgs::msg::AtsState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ats_msgs::msg::AtsState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ats_msgs::msg::AtsState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ats_msgs::msg::AtsState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ats_msgs::msg::AtsState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ats_msgs::msg::AtsState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ats_msgs::msg::AtsState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ats_msgs__msg__AtsState
    std::shared_ptr<ats_msgs::msg::AtsState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ats_msgs__msg__AtsState
    std::shared_ptr<ats_msgs::msg::AtsState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const AtsState_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->system_state != other.system_state) {
      return false;
    }
    if (this->pose_frame != other.pose_frame) {
      return false;
    }
    if (this->pose_x != other.pose_x) {
      return false;
    }
    if (this->pose_y != other.pose_y) {
      return false;
    }
    if (this->pose_yaw != other.pose_yaw) {
      return false;
    }
    if (this->vel_vx != other.vel_vx) {
      return false;
    }
    if (this->vel_vy != other.vel_vy) {
      return false;
    }
    if (this->vel_wz != other.vel_wz) {
      return false;
    }
    if (this->battery_soc != other.battery_soc) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->primary_id != other.primary_id) {
      return false;
    }
    if (this->lost_sec != other.lost_sec) {
      return false;
    }
    if (this->vision_json != other.vision_json) {
      return false;
    }
    if (this->plan_json != other.plan_json) {
      return false;
    }
    if (this->current_index != other.current_index) {
      return false;
    }
    if (this->queue_status != other.queue_status) {
      return false;
    }
    if (this->history_json != other.history_json) {
      return false;
    }
    if (this->roe_ok != other.roe_ok) {
      return false;
    }
    if (this->safe_backstop != other.safe_backstop) {
      return false;
    }
    if (this->max_speed != other.max_speed) {
      return false;
    }
    if (this->last_violation != other.last_violation) {
      return false;
    }
    if (this->events_json != other.events_json) {
      return false;
    }
    return true;
  }
  bool operator!=(const AtsState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct AtsState_

// alias to use template instance with default allocator
using AtsState =
  ats_msgs::msg::AtsState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ats_msgs

#endif  // ATS_MSGS__MSG__DETAIL__ATS_STATE__STRUCT_HPP_
