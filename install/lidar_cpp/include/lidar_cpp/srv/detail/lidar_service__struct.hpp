// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from lidar_cpp:srv/LidarService.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__STRUCT_HPP_
#define LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__STRUCT_HPP_

#include <rosidl_runtime_cpp/bounded_vector.hpp>
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>


#ifndef _WIN32
# define DEPRECATED__lidar_cpp__srv__LidarService_Request __attribute__((deprecated))
#else
# define DEPRECATED__lidar_cpp__srv__LidarService_Request __declspec(deprecated)
#endif

namespace lidar_cpp
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LidarService_Request_
{
  using Type = LidarService_Request_<ContainerAllocator>;

  explicit LidarService_Request_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  explicit LidarService_Request_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->structure_needs_at_least_one_member = 0;
    }
  }

  // field types and members
  using _structure_needs_at_least_one_member_type =
    uint8_t;
  _structure_needs_at_least_one_member_type structure_needs_at_least_one_member;


  // constant declarations

  // pointer types
  using RawPtr =
    lidar_cpp::srv::LidarService_Request_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_cpp::srv::LidarService_Request_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_cpp::srv::LidarService_Request_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_cpp::srv::LidarService_Request_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_cpp::srv::LidarService_Request_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_cpp::srv::LidarService_Request_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_cpp::srv::LidarService_Request_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_cpp::srv::LidarService_Request_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_cpp::srv::LidarService_Request_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_cpp::srv::LidarService_Request_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_cpp__srv__LidarService_Request
    std::shared_ptr<lidar_cpp::srv::LidarService_Request_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_cpp__srv__LidarService_Request
    std::shared_ptr<lidar_cpp::srv::LidarService_Request_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarService_Request_ & other) const
  {
    if (this->structure_needs_at_least_one_member != other.structure_needs_at_least_one_member) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarService_Request_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarService_Request_

// alias to use template instance with default allocator
using LidarService_Request =
  lidar_cpp::srv::LidarService_Request_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace lidar_cpp


// Include directives for member types
// Member 'pcl_response'
#include "sensor_msgs/msg/detail/point_cloud2__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__lidar_cpp__srv__LidarService_Response __attribute__((deprecated))
#else
# define DEPRECATED__lidar_cpp__srv__LidarService_Response __declspec(deprecated)
#endif

namespace lidar_cpp
{

namespace srv
{

// message struct
template<class ContainerAllocator>
struct LidarService_Response_
{
  using Type = LidarService_Response_<ContainerAllocator>;

  explicit LidarService_Response_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pcl_response(_init)
  {
    (void)_init;
  }

  explicit LidarService_Response_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : pcl_response(_alloc, _init)
  {
    (void)_init;
  }

  // field types and members
  using _pcl_response_type =
    sensor_msgs::msg::PointCloud2_<ContainerAllocator>;
  _pcl_response_type pcl_response;

  // setters for named parameter idiom
  Type & set__pcl_response(
    const sensor_msgs::msg::PointCloud2_<ContainerAllocator> & _arg)
  {
    this->pcl_response = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    lidar_cpp::srv::LidarService_Response_<ContainerAllocator> *;
  using ConstRawPtr =
    const lidar_cpp::srv::LidarService_Response_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<lidar_cpp::srv::LidarService_Response_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<lidar_cpp::srv::LidarService_Response_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      lidar_cpp::srv::LidarService_Response_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<lidar_cpp::srv::LidarService_Response_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      lidar_cpp::srv::LidarService_Response_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<lidar_cpp::srv::LidarService_Response_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<lidar_cpp::srv::LidarService_Response_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<lidar_cpp::srv::LidarService_Response_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__lidar_cpp__srv__LidarService_Response
    std::shared_ptr<lidar_cpp::srv::LidarService_Response_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__lidar_cpp__srv__LidarService_Response
    std::shared_ptr<lidar_cpp::srv::LidarService_Response_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const LidarService_Response_ & other) const
  {
    if (this->pcl_response != other.pcl_response) {
      return false;
    }
    return true;
  }
  bool operator!=(const LidarService_Response_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct LidarService_Response_

// alias to use template instance with default allocator
using LidarService_Response =
  lidar_cpp::srv::LidarService_Response_<std::allocator<void>>;

// constant definitions

}  // namespace srv

}  // namespace lidar_cpp

namespace lidar_cpp
{

namespace srv
{

struct LidarService
{
  using Request = lidar_cpp::srv::LidarService_Request;
  using Response = lidar_cpp::srv::LidarService_Response;
};

}  // namespace srv

}  // namespace lidar_cpp

#endif  // LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__STRUCT_HPP_
