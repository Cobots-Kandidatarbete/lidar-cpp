# generated from
# rosidl_cmake/cmake/template/rosidl_cmake_export_typesupport_targets.cmake.in

set(_exported_typesupport_targets
  "__rosidl_typesupport_introspection_c:lidar_cpp__rosidl_typesupport_introspection_c;__rosidl_typesupport_introspection_cpp:lidar_cpp__rosidl_typesupport_introspection_cpp")

# populate lidar_cpp_TARGETS_<suffix>
if(NOT _exported_typesupport_targets STREQUAL "")
  # loop over typesupport targets
  foreach(_tuple ${_exported_typesupport_targets})
    string(REPLACE ":" ";" _tuple "${_tuple}")
    list(GET _tuple 0 _suffix)
    list(GET _tuple 1 _target)

    set(_target "lidar_cpp::${_target}")
    if(NOT TARGET "${_target}")
      # the exported target must exist
      message(WARNING "Package 'lidar_cpp' exports the typesupport target '${_target}' which doesn't exist")
    else()
      list(APPEND lidar_cpp_TARGETS${_suffix} "${_target}")
    endif()
  endforeach()
endif()
