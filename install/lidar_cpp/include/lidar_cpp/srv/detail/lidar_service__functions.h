// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from lidar_cpp:srv/LidarService.idl
// generated code does not contain a copyright notice

#ifndef LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__FUNCTIONS_H_
#define LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "lidar_cpp/msg/rosidl_generator_c__visibility_control.h"

#include "lidar_cpp/srv/detail/lidar_service__struct.h"

/// Initialize srv/LidarService message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lidar_cpp__srv__LidarService_Request
 * )) before or use
 * lidar_cpp__srv__LidarService_Request__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
bool
lidar_cpp__srv__LidarService_Request__init(lidar_cpp__srv__LidarService_Request * msg);

/// Finalize srv/LidarService message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
void
lidar_cpp__srv__LidarService_Request__fini(lidar_cpp__srv__LidarService_Request * msg);

/// Create srv/LidarService message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lidar_cpp__srv__LidarService_Request__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
lidar_cpp__srv__LidarService_Request *
lidar_cpp__srv__LidarService_Request__create();

/// Destroy srv/LidarService message.
/**
 * It calls
 * lidar_cpp__srv__LidarService_Request__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
void
lidar_cpp__srv__LidarService_Request__destroy(lidar_cpp__srv__LidarService_Request * msg);


/// Initialize array of srv/LidarService messages.
/**
 * It allocates the memory for the number of elements and calls
 * lidar_cpp__srv__LidarService_Request__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
bool
lidar_cpp__srv__LidarService_Request__Sequence__init(lidar_cpp__srv__LidarService_Request__Sequence * array, size_t size);

/// Finalize array of srv/LidarService messages.
/**
 * It calls
 * lidar_cpp__srv__LidarService_Request__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
void
lidar_cpp__srv__LidarService_Request__Sequence__fini(lidar_cpp__srv__LidarService_Request__Sequence * array);

/// Create array of srv/LidarService messages.
/**
 * It allocates the memory for the array and calls
 * lidar_cpp__srv__LidarService_Request__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
lidar_cpp__srv__LidarService_Request__Sequence *
lidar_cpp__srv__LidarService_Request__Sequence__create(size_t size);

/// Destroy array of srv/LidarService messages.
/**
 * It calls
 * lidar_cpp__srv__LidarService_Request__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
void
lidar_cpp__srv__LidarService_Request__Sequence__destroy(lidar_cpp__srv__LidarService_Request__Sequence * array);

/// Initialize srv/LidarService message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * lidar_cpp__srv__LidarService_Response
 * )) before or use
 * lidar_cpp__srv__LidarService_Response__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
bool
lidar_cpp__srv__LidarService_Response__init(lidar_cpp__srv__LidarService_Response * msg);

/// Finalize srv/LidarService message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
void
lidar_cpp__srv__LidarService_Response__fini(lidar_cpp__srv__LidarService_Response * msg);

/// Create srv/LidarService message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * lidar_cpp__srv__LidarService_Response__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
lidar_cpp__srv__LidarService_Response *
lidar_cpp__srv__LidarService_Response__create();

/// Destroy srv/LidarService message.
/**
 * It calls
 * lidar_cpp__srv__LidarService_Response__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
void
lidar_cpp__srv__LidarService_Response__destroy(lidar_cpp__srv__LidarService_Response * msg);


/// Initialize array of srv/LidarService messages.
/**
 * It allocates the memory for the number of elements and calls
 * lidar_cpp__srv__LidarService_Response__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
bool
lidar_cpp__srv__LidarService_Response__Sequence__init(lidar_cpp__srv__LidarService_Response__Sequence * array, size_t size);

/// Finalize array of srv/LidarService messages.
/**
 * It calls
 * lidar_cpp__srv__LidarService_Response__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
void
lidar_cpp__srv__LidarService_Response__Sequence__fini(lidar_cpp__srv__LidarService_Response__Sequence * array);

/// Create array of srv/LidarService messages.
/**
 * It allocates the memory for the array and calls
 * lidar_cpp__srv__LidarService_Response__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
lidar_cpp__srv__LidarService_Response__Sequence *
lidar_cpp__srv__LidarService_Response__Sequence__create(size_t size);

/// Destroy array of srv/LidarService messages.
/**
 * It calls
 * lidar_cpp__srv__LidarService_Response__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_lidar_cpp
void
lidar_cpp__srv__LidarService_Response__Sequence__destroy(lidar_cpp__srv__LidarService_Response__Sequence * array);

#ifdef __cplusplus
}
#endif

#endif  // LIDAR_CPP__SRV__DETAIL__LIDAR_SERVICE__FUNCTIONS_H_
