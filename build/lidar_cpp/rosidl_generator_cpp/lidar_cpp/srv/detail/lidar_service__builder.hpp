// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from lidar_cpp:srv/LidarService.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__BUILDER_HPP_
#define LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__BUILDER_HPP_

#include "lidar_cpp/srv/detail/lidar_service__struct.hpp"
#include <rosidl_runtime_cpp/message_initialization.hpp>
#include <algorithm>
#include <utility>


namespace lidar_cpp
{

namespace srv
{


}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_cpp::srv::LidarService_Request>()
{
  return ::lidar_cpp::srv::LidarService_Request(rosidl_runtime_cpp::MessageInitialization::ZERO);
}

}  // namespace lidar_cpp


namespace lidar_cpp
{

namespace srv
{

namespace builder
{

class Init_LidarService_Response_pcl_response
{
public:
  Init_LidarService_Response_pcl_response()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::lidar_cpp::srv::LidarService_Response pcl_response(::lidar_cpp::srv::LidarService_Response::_pcl_response_type arg)
  {
    msg_.pcl_response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::lidar_cpp::srv::LidarService_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::lidar_cpp::srv::LidarService_Response>()
{
  return lidar_cpp::srv::builder::Init_LidarService_Response_pcl_response();
}

}  // namespace lidar_cpp

#endif  // LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__BUILDER_HPP_
