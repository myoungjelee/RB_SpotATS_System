// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ats_msgs:msg/PlanCommand.idl
// generated code does not contain a copyright notice

#ifndef ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__STRUCT_HPP_
#define ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__STRUCT_HPP_

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
# define DEPRECATED__ats_msgs__msg__PlanCommand __attribute__((deprecated))
#else
# define DEPRECATED__ats_msgs__msg__PlanCommand __declspec(deprecated)
#endif

namespace ats_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct PlanCommand_
{
  using Type = PlanCommand_<ContainerAllocator>;

  explicit PlanCommand_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_id = "";
      this->plan_json = "";
    }
  }

  explicit PlanCommand_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : stamp(_alloc, _init),
    mission_id(_alloc),
    plan_json(_alloc)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mission_id = "";
      this->plan_json = "";
    }
  }

  // field types and members
  using _stamp_type =
    builtin_interfaces::msg::Time_<ContainerAllocator>;
  _stamp_type stamp;
  using _mission_id_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _mission_id_type mission_id;
  using _plan_json_type =
    std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>>;
  _plan_json_type plan_json;

  // setters for named parameter idiom
  Type & set__stamp(
    const builtin_interfaces::msg::Time_<ContainerAllocator> & _arg)
  {
    this->stamp = _arg;
    return *this;
  }
  Type & set__mission_id(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->mission_id = _arg;
    return *this;
  }
  Type & set__plan_json(
    const std::basic_string<char, std::char_traits<char>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<char>> & _arg)
  {
    this->plan_json = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ats_msgs::msg::PlanCommand_<ContainerAllocator> *;
  using ConstRawPtr =
    const ats_msgs::msg::PlanCommand_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ats_msgs::msg::PlanCommand_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ats_msgs::msg::PlanCommand_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ats_msgs::msg::PlanCommand_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ats_msgs::msg::PlanCommand_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ats_msgs::msg::PlanCommand_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ats_msgs::msg::PlanCommand_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ats_msgs::msg::PlanCommand_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ats_msgs::msg::PlanCommand_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ats_msgs__msg__PlanCommand
    std::shared_ptr<ats_msgs::msg::PlanCommand_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ats_msgs__msg__PlanCommand
    std::shared_ptr<ats_msgs::msg::PlanCommand_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const PlanCommand_ & other) const
  {
    if (this->stamp != other.stamp) {
      return false;
    }
    if (this->mission_id != other.mission_id) {
      return false;
    }
    if (this->plan_json != other.plan_json) {
      return false;
    }
    return true;
  }
  bool operator!=(const PlanCommand_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct PlanCommand_

// alias to use template instance with default allocator
using PlanCommand =
  ats_msgs::msg::PlanCommand_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ats_msgs

#endif  // ATS_MSGS__MSG__DETAIL__PLAN_COMMAND__STRUCT_HPP_
