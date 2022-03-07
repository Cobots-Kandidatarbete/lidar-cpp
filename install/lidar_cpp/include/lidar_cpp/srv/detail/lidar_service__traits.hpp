// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from lidar_cpp:srv/LidarService.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__TRAITS_HPP_
#define LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__TRAITS_HPP_

#include "lidar_cpp/srv/detail/lidar_service__struct.hpp"
#include <stdint.h>
#include <rosidl_runtime_cpp/traits.hpp>
#include <sstream>
#include <string>
#include <type_traits>

namespace rosidl_generator_traits
{

inline void to_yaml(
  const lidar_cpp::srv::LidarService_Request & msg,
  std::ostream & out, size_t indentation = 0)
{
  (void)msg;
  (void)indentation;
  out << "null\n";
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const lidar_cpp::srv::LidarService_Request & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<lidar_cpp::srv::LidarService_Request>()
{
  return "lidar_cpp::srv::LidarService_Request";
}

template<>
inline const char * name<lidar_cpp::srv::LidarService_Request>()
{
  return "lidar_cpp/srv/LidarService_Request";
}

template<>
struct has_fixed_size<lidar_cpp::srv::LidarService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<lidar_cpp::srv::LidarService_Request>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<lidar_cpp::srv::LidarService_Request>
  : std::true_type {};

}  // namespace rosidl_generator_traits

// Include directives for member types
// Member 'pcl_response'
#include "sensor_msgs/msg/detail/point_cloud2__traits.hpp"

namespace rosidl_generator_traits
{

inline void to_yaml(
  const lidar_cpp::srv::LidarService_Response & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: pcl_response
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pcl_response:\n";
    to_yaml(msg.pcl_response, out, indentation + 2);
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const lidar_cpp::srv::LidarService_Response & msg)
{
  std::ostringstream out;
  to_yaml(msg, out);
  return out.str();
}

template<>
inline const char * data_type<lidar_cpp::srv::LidarService_Response>()
{
  return "lidar_cpp::srv::LidarService_Response";
}

template<>
inline const char * name<lidar_cpp::srv::LidarService_Response>()
{
  return "lidar_cpp/srv/LidarService_Response";
}

template<>
struct has_fixed_size<lidar_cpp::srv::LidarService_Response>
  : std::integral_constant<bool, has_fixed_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct has_bounded_size<lidar_cpp::srv::LidarService_Response>
  : std::integral_constant<bool, has_bounded_size<sensor_msgs::msg::PointCloud2>::value> {};

template<>
struct is_message<lidar_cpp::srv::LidarService_Response>
  : std::true_type {};

}  // namespace rosidl_generator_traits

namespace rosidl_generator_traits
{

template<>
inline const char * data_type<lidar_cpp::srv::LidarService>()
{
  return "lidar_cpp::srv::LidarService";
}

template<>
inline const char * name<lidar_cpp::srv::LidarService>()
{
  return "lidar_cpp/srv/LidarService";
}

template<>
struct has_fixed_size<lidar_cpp::srv::LidarService>
  : std::integral_constant<
    bool,
    has_fixed_size<lidar_cpp::srv::LidarService_Request>::value &&
    has_fixed_size<lidar_cpp::srv::LidarService_Response>::value
  >
{
};

template<>
struct has_bounded_size<lidar_cpp::srv::LidarService>
  : std::integral_constant<
    bool,
    has_bounded_size<lidar_cpp::srv::LidarService_Request>::value &&
    has_bounded_size<lidar_cpp::srv::LidarService_Response>::value
  >
{
};

template<>
struct is_service<lidar_cpp::srv::LidarService>
  : std::true_type
{
};

template<>
struct is_service_request<lidar_cpp::srv::LidarService_Request>
  : std::true_type
{
};

template<>
struct is_service_response<lidar_cpp::srv::LidarService_Response>
  : std::true_type
{
};

}  // namespace rosidl_generator_traits

#endif  // LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__TRAITS_HPP_
