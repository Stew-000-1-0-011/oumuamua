# Injected into every package via CMAKE_PROJECT_INCLUDE (see colcon.meta / colcon defaults).
# Some packages hard-code an older CMAKE_CXX_STANDARD after project(), which would
# shadow -DCMAKE_CXX_STANDARD on the command line. To keep the whole workspace on
# one standard, re-apply OUMUAMUA_CXX_STANDARD to every target at the end of the
# top-level directory scope.
if(NOT DEFINED OUMUAMUA_CXX_STANDARD)
  return()
endif()

function(_oumuamua_apply_cxx_standard dir)
  get_property(_targets DIRECTORY "${dir}" PROPERTY BUILDSYSTEM_TARGETS)
  foreach(_t IN LISTS _targets)
    get_target_property(_type ${_t} TYPE)
    if(NOT _type STREQUAL "INTERFACE_LIBRARY" AND NOT _type STREQUAL "UTILITY")
      set_target_properties(${_t} PROPERTIES
        CXX_STANDARD ${OUMUAMUA_CXX_STANDARD}
        CXX_STANDARD_REQUIRED ON)
    endif()
  endforeach()
  get_property(_subdirs DIRECTORY "${dir}" PROPERTY SUBDIRECTORIES)
  foreach(_d IN LISTS _subdirs)
    _oumuamua_apply_cxx_standard("${_d}")
  endforeach()
endfunction()

function(_oumuamua_apply_cxx_standard_root)
  _oumuamua_apply_cxx_standard("${CMAKE_SOURCE_DIR}")
endfunction()

cmake_language(DEFER DIRECTORY "${CMAKE_SOURCE_DIR}" CALL _oumuamua_apply_cxx_standard_root)
