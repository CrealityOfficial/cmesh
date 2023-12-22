__cc_find(trimesh2)
__assert_target(trimesh2)
__cc_find(eigen)
__assert_target(eigen)

__cc_find(cgal)
__assert_target(cgal)

__cc_find(tbb)
__assert_target(tbb)

__cc_find(boost_static)

__cc_find(clipper)
__assert_target(clipper)

__enable_cxx14()

if (MSVC)
  add_definitions(/bigobj)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif ()

add_definitions( -DNOMINMAX -DCGAL_LINKED_WITH_TBB)

__files_group(${CMAKE_CURRENT_SOURCE_DIR}/include/cmesh/ EXTERNAL_SRCS)
__files_group_2(${CMAKE_CURRENT_SOURCE_DIR}/impl/ internal INTERNAL_SRCS)
set(SRCS ${EXTERNAL_SRCS} ${INTERNAL_SRCS})

set(INCS ${EIGEN_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/ ${CGAL_INCLUDE_DIRS})
set(INTERFACES ${CMAKE_CURRENT_SOURCE_DIR}/include/)

set(LIBS clipper boost_system tbb eigen cgal)
set(ILIBS trimesh2)
set(DEFS CGAL_NO_GMP=0 CGAL_NO_MPFR=0  CGAL_EIGEN3_ENABLED CGAL_HEADER_ONLY __TBBMALLOC_NO_IMPLICIT_LINKAGE=1)

__add_common_library(cmesh_noigl)

add_definitions(-DCGAL_NO_GMP=0)
add_definitions(-DCGAL_NO_MPFR=0)

if(CC_INSTALL_CMESH_NOIGL)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/include/ cmesh_noigl)
endif()
