// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from lidar_cpp:srv/LidarService.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__STRUCT_H_
#define LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Struct defined in srv/LidarService in the package lidar_cpp.
typedef struct lidar_cpp__srv__LidarService_Request
{
  uint8_t structure_needs_at_least_one_member;
} lidar_cpp__srv__LidarService_Request;

// Struct for a sequence of lidar_cpp__srv__LidarService_Request.
typedef struct lidar_cpp__srv__LidarService_Request__Sequence
{
  lidar_cpp__srv__LidarService_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_cpp__srv__LidarService_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'pcl_response'
#include "sensor_msgs/msg/detail/point_cloud2__struct.h"

// Struct defined in srv/LidarService in the package lidar_cpp.
typedef struct lidar_cpp__srv__LidarService_Response
{
  sensor_msgs__msg__PointCloud2 pcl_response;
} lidar_cpp__srv__LidarService_Response;

// Struct for a sequence of lidar_cpp__srv__LidarService_Response.
typedef struct lidar_cpp__srv__LidarService_Response__Sequence
{
  lidar_cpp__srv__LidarService_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} lidar_cpp__srv__LidarService_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__STRUCT_H_
