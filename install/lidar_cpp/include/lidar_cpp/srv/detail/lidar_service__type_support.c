// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from lidar_cpp:srv/LidarService.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "lidar_cpp/srv/detail/lidar_service__rosidl_typesupport_introspection_c.h"
#include "lidar_cpp/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "lidar_cpp/srv/detail/lidar_service__functions.h"
#include "lidar_cpp/srv/detail/lidar_service__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_cpp__srv__LidarService_Request__init(message_memory);
}

void LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_fini_function(void * message_memory)
{
  lidar_cpp__srv__LidarService_Request__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_message_member_array[1] = {
  {
    "structure_needs_at_least_one_member",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_cpp__srv__LidarService_Request, structure_needs_at_least_one_member),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_message_members = {
  "lidar_cpp__srv",  // message namespace
  "LidarService_Request",  // message name
  1,  // number of fields
  sizeof(lidar_cpp__srv__LidarService_Request),
  LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_message_member_array,  // message members
  LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_init_function,  // function to initialize message memory (memory has to be allocated)
  LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_message_type_support_handle = {
  0,
  &LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_cpp
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_cpp, srv, LidarService_Request)() {
  if (!LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_message_type_support_handle.typesupport_identifier) {
    LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &LidarService_Request__rosidl_typesupport_introspection_c__LidarService_Request_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

// already included above
// #include <stddef.h>
// already included above
// #include "lidar_cpp/srv/detail/lidar_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "lidar_cpp/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "rosidl_typesupport_introspection_c/field_types.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
// already included above
// #include "rosidl_typesupport_introspection_c/message_introspection.h"
// already included above
// #include "lidar_cpp/srv/detail/lidar_service__functions.h"
// already included above
// #include "lidar_cpp/srv/detail/lidar_service__struct.h"


// Include directives for member types
// Member `pcl_response`
#include "sensor_msgs/msg/point_cloud2.h"
// Member `pcl_response`
#include "sensor_msgs/msg/detail/point_cloud2__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  lidar_cpp__srv__LidarService_Response__init(message_memory);
}

void LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_fini_function(void * message_memory)
{
  lidar_cpp__srv__LidarService_Response__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_member_array[1] = {
  {
    "pcl_response",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(lidar_cpp__srv__LidarService_Response, pcl_response),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_members = {
  "lidar_cpp__srv",  // message namespace
  "LidarService_Response",  // message name
  1,  // number of fields
  sizeof(lidar_cpp__srv__LidarService_Response),
  LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_member_array,  // message members
  LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_init_function,  // function to initialize message memory (memory has to be allocated)
  LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_type_support_handle = {
  0,
  &LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_cpp
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_cpp, srv, LidarService_Response)() {
  LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, sensor_msgs, msg, PointCloud2)();
  if (!LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_type_support_handle.typesupport_identifier) {
    LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &LidarService_Response__rosidl_typesupport_introspection_c__LidarService_Response_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif

#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "lidar_cpp/msg/rosidl_typesupport_introspection_c__visibility_control.h"
// already included above
// #include "lidar_cpp/srv/detail/lidar_service__rosidl_typesupport_introspection_c.h"
// already included above
// #include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/service_introspection.h"

// this is intentionally not const to allow initialization later to prevent an initialization race
static rosidl_typesupport_introspection_c__ServiceMembers lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_service_members = {
  "lidar_cpp__srv",  // service namespace
  "LidarService",  // service name
  // these two fields are initialized below on the first access
  NULL,  // request message
  // lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_Request_message_type_support_handle,
  NULL  // response message
  // lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_Response_message_type_support_handle
};

static rosidl_service_type_support_t lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_service_type_support_handle = {
  0,
  &lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_service_members,
  get_service_typesupport_handle_function,
};

// Forward declaration of request/response type support functions
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_cpp, srv, LidarService_Request)();

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_cpp, srv, LidarService_Response)();

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_lidar_cpp
const rosidl_service_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_cpp, srv, LidarService)() {
  if (!lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_service_type_support_handle.typesupport_identifier) {
    lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_service_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  rosidl_typesupport_introspection_c__ServiceMembers * service_members =
    (rosidl_typesupport_introspection_c__ServiceMembers *)lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_service_type_support_handle.data;

  if (!service_members->request_members_) {
    service_members->request_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_cpp, srv, LidarService_Request)()->data;
  }
  if (!service_members->response_members_) {
    service_members->response_members_ =
      (const rosidl_typesupport_introspection_c__MessageMembers *)
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, lidar_cpp, srv, LidarService_Response)()->data;
  }

  return &lidar_cpp__srv__detail__lidar_service__rosidl_typesupport_introspection_c__LidarService_service_type_support_handle;
}
