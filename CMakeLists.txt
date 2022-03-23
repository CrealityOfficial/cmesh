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

set(SRCS include/cmesh/interface.h
		 include/cmesh/poly/roof.h
		 include/cmesh/mesh/boolean.h
		 include/cmesh/mesh/richmesh.h
		 include/cmesh/mesh/repair.h
		 
		 impl/roof.cpp
		 impl/boolean.cpp
		 impl/richmesh.cpp
		 impl/richmeshimpl.h
		 impl/richmeshimpl.cpp
		 impl/ctype.h
		 impl/cboolean.cpp
		 impl/cboolean.h
		 impl/cconversion.h
		 impl/cconversion.cpp
		 impl/repair.cpp
		 impl/cholefill.h
		 impl/cholefill.cpp
		 )
		

set(INCS ${EIGEN_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/include/)
set(INTERFACES ${CMAKE_CURRENT_SOURCE_DIR}/include/)
set(LIBS trimesh2 clipper boost_system)
set(DEFS CGAL_NO_GMP=1 CGAL_NO_MPFR=1 CGAL_EIGEN3_ENABLED CGAL_HEADER_ONLY)

__add_platform_library(cmesh)

if(CC_INSTALL_CMESH)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/include/ cmesh)
endif()
