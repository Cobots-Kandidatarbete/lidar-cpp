// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from lidar_cpp:srv/LidarService.idl
// generated code does not contain a copyright notice
#include "lidar_cpp/srv/detail/lidar_service__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

bool
lidar_cpp__srv__LidarService_Request__init(lidar_cpp__srv__LidarService_Request * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
lidar_cpp__srv__LidarService_Request__fini(lidar_cpp__srv__LidarService_Request * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

lidar_cpp__srv__LidarService_Request *
lidar_cpp__srv__LidarService_Request__create()
{
  lidar_cpp__srv__LidarService_Request * msg = (lidar_cpp__srv__LidarService_Request *)malloc(sizeof(lidar_cpp__srv__LidarService_Request));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_cpp__srv__LidarService_Request));
  bool success = lidar_cpp__srv__LidarService_Request__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
lidar_cpp__srv__LidarService_Request__destroy(lidar_cpp__srv__LidarService_Request * msg)
{
  if (msg) {
    lidar_cpp__srv__LidarService_Request__fini(msg);
  }
  free(msg);
}


bool
lidar_cpp__srv__LidarService_Request__Sequence__init(lidar_cpp__srv__LidarService_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  lidar_cpp__srv__LidarService_Request * data = NULL;
  if (size) {
    data = (lidar_cpp__srv__LidarService_Request *)calloc(size, sizeof(lidar_cpp__srv__LidarService_Request));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_cpp__srv__LidarService_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_cpp__srv__LidarService_Request__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
lidar_cpp__srv__LidarService_Request__Sequence__fini(lidar_cpp__srv__LidarService_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lidar_cpp__srv__LidarService_Request__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

lidar_cpp__srv__LidarService_Request__Sequence *
lidar_cpp__srv__LidarService_Request__Sequence__create(size_t size)
{
  lidar_cpp__srv__LidarService_Request__Sequence * array = (lidar_cpp__srv__LidarService_Request__Sequence *)malloc(sizeof(lidar_cpp__srv__LidarService_Request__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = lidar_cpp__srv__LidarService_Request__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
lidar_cpp__srv__LidarService_Request__Sequence__destroy(lidar_cpp__srv__LidarService_Request__Sequence * array)
{
  if (array) {
    lidar_cpp__srv__LidarService_Request__Sequence__fini(array);
  }
  free(array);
}


// Include directives for member types
// Member `pcl_response`
#include "sensor_msgs/msg/detail/point_cloud2__functions.h"

bool
lidar_cpp__srv__LidarService_Response__init(lidar_cpp__srv__LidarService_Response * msg)
{
  if (!msg) {
    return false;
  }
  // pcl_response
  if (!sensor_msgs__msg__PointCloud2__init(&msg->pcl_response)) {
    lidar_cpp__srv__LidarService_Response__fini(msg);
    return false;
  }
  return true;
}

void
lidar_cpp__srv__LidarService_Response__fini(lidar_cpp__srv__LidarService_Response * msg)
{
  if (!msg) {
    return;
  }
  // pcl_response
  sensor_msgs__msg__PointCloud2__fini(&msg->pcl_response);
}

lidar_cpp__srv__LidarService_Response *
lidar_cpp__srv__LidarService_Response__create()
{
  lidar_cpp__srv__LidarService_Response * msg = (lidar_cpp__srv__LidarService_Response *)malloc(sizeof(lidar_cpp__srv__LidarService_Response));
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(lidar_cpp__srv__LidarService_Response));
  bool success = lidar_cpp__srv__LidarService_Response__init(msg);
  if (!success) {
    free(msg);
    return NULL;
  }
  return msg;
}

void
lidar_cpp__srv__LidarService_Response__destroy(lidar_cpp__srv__LidarService_Response * msg)
{
  if (msg) {
    lidar_cpp__srv__LidarService_Response__fini(msg);
  }
  free(msg);
}


bool
lidar_cpp__srv__LidarService_Response__Sequence__init(lidar_cpp__srv__LidarService_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  lidar_cpp__srv__LidarService_Response * data = NULL;
  if (size) {
    data = (lidar_cpp__srv__LidarService_Response *)calloc(size, sizeof(lidar_cpp__srv__LidarService_Response));
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = lidar_cpp__srv__LidarService_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        lidar_cpp__srv__LidarService_Response__fini(&data[i - 1]);
      }
      free(data);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
lidar_cpp__srv__LidarService_Response__Sequence__fini(lidar_cpp__srv__LidarService_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      lidar_cpp__srv__LidarService_Response__fini(&array->data[i]);
    }
    free(array->data);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

lidar_cpp__srv__LidarService_Response__Sequence *
lidar_cpp__srv__LidarService_Response__Sequence__create(size_t size)
{
  lidar_cpp__srv__LidarService_Response__Sequence * array = (lidar_cpp__srv__LidarService_Response__Sequence *)malloc(sizeof(lidar_cpp__srv__LidarService_Response__Sequence));
  if (!array) {
    return NULL;
  }
  bool success = lidar_cpp__srv__LidarService_Response__Sequence__init(array, size);
  if (!success) {
    free(array);
    return NULL;
  }
  return array;
}

void
lidar_cpp__srv__LidarService_Response__Sequence__destroy(lidar_cpp__srv__LidarService_Response__Sequence * array)
{
  if (array) {
    lidar_cpp__srv__LidarService_Response__Sequence__fini(array);
  }
  free(array);
}
