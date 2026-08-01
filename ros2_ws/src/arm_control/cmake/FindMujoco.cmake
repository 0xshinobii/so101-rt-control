# Minimal locator for the prebuilt MuJoCo C library laid out as
#   $MUJOCO_DIR/include/mujoco/mujoco.h  and  $MUJOCO_DIR/lib/libmujoco.so
# (the layout of the official linux release tarball). Defines an imported
# target `mujoco::mujoco` so consumers just `target_link_libraries(... mujoco::mujoco)`.
if(NOT DEFINED MUJOCO_DIR AND DEFINED ENV{MUJOCO_DIR})
  set(MUJOCO_DIR $ENV{MUJOCO_DIR})
endif()

find_path(MUJOCO_INCLUDE_DIR
  NAMES mujoco/mujoco.h
  HINTS ${MUJOCO_DIR}
  PATH_SUFFIXES include
)

find_library(MUJOCO_LIBRARY
  NAMES mujoco
  HINTS ${MUJOCO_DIR}
  PATH_SUFFIXES lib
)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Mujoco
  REQUIRED_VARS MUJOCO_LIBRARY MUJOCO_INCLUDE_DIR
)

if(Mujoco_FOUND AND NOT TARGET mujoco::mujoco)
  add_library(mujoco::mujoco UNKNOWN IMPORTED)
  set_target_properties(mujoco::mujoco PROPERTIES
    IMPORTED_LOCATION "${MUJOCO_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${MUJOCO_INCLUDE_DIR}"
  )
endif()
