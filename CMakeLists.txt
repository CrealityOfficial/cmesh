__assert_target(trimesh2)

__cc_find(Boost)
__cc_find(Eigen)
__assert_target(boost_system)

if(NOT EIGEN_INCLUDE_DIRS)
	message(FATAL_ERROR "Please Specified EIGEN_INCLUDE_DIRS")
endif()

if (MSVC)
  add_definitions(/bigobj)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif ()

__cc_find(CGAL)

set(SRCS cmesh/interface.h
		 cmesh/poly/roof.h
		 cmesh/mesh/boolean.h
		 
		 impl/roof.cpp
		 impl/boolean.cpp
		 )
		

set(INCS ${EIGEN_INCLUDE_DIRS})
set(LIBS trimesh2 clipper boost_system)
set(DEFS CGAL_NO_GMP=1 CGAL_NO_MPFR=1 CGAL_EIGEN3_ENABLED CGAL_HEADER_ONLY)

__add_platform_library(cmesh)

if(CC_INSTALL_CMESH)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/cmesh cmesh)
endif()
