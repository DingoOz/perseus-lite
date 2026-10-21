# INCLUDE POSSIBLY LOCAL DEPENDENCIES

# check if libraries are already present (installed packages), if not, build
# them in-tree
find_package(simple_networking QUIET)
if(NOT simple_networking_FOUND)
  message(STATUS "simple_networking not found, adding subdirectory...")
  # add the directory with its CMakeLists.txt
  add_subdirectory(../../../shared/simple-networking simple-networking)
endif()
