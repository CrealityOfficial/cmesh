__assert_target(trimesh2)

if(NOT EIGEN_INCLUDE_DIRS)
	message(FATAL_ERROR "Please Specified EIGEN_INCLUDE_DIRS")
endif()

if(NOT BOOST_INCLUDE_DIRS)
	message(FATAL_ERROR "Please Specified BOOST_INCLUDE_DIRS")
endif()

if (MSVC)
  add_definitions(/bigobj)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif ()

__cc_find(CGAL)
#include(CGAL)
#
#__cgal_include_package(Straight_skeleton_2)
#__cgal_include_package(Polygon)
#__cgal_include_package(Polygon_mesh_processing)
#__cgal_include_package(Polyhedron)
#__cgal_include_package(Polyhedron_IO)
#__cgal_include_package(Random_numbers)
#__cgal_include_package(Box_intersection_d) 
#__cgal_include_package(AABB_tree)
#__cgal_include_package(Spatial_searching)
#__cgal_include_package(Spatial_sorting)
#__cgal_include_package(Generator)
#__cgal_include_package(Union_find)
#__cgal_include_package(Modifier)
#__cgal_include_package(Triangulation_2)
#__cgal_include_package(TDS_2)
#__cgal_include_package(Surface_mesh)
#__cgal_include_package(Surface_mesh_simplification)
#
##enable cluster point 
#__cgal_include_package(Point_set_processing_3)
#__cgal_include_package(Point_set_3)
#__cgal_include_package(Solver_interface)
#__cgal_include_package(Triangulation_3)
#__cgal_include_package(TDS_3)
#__cgal_include_package(Subdivision_method_3)
#__cgal_include_package(Principal_component_analysis_LGPL)

set(SRCS cmesh/interface.h
		 cmesh/mesh/simplify.h
		 cmesh/mesh/simplify.cpp
		 cmesh/mesh/roof.h
		 cmesh/mesh/roof.cpp
		 cmesh/mesh/boolean.h
		 cmesh/mesh/boolean.cpp
		 cmesh/mesh/meshgeneration.h
		 cmesh/mesh/meshgeneration.cpp
		 cmesh/mesh/meshwrapper.h
		 cmesh/mesh/meshwrapper.cpp
		 cmesh/mesh/meshmeshing.h
		 cmesh/mesh/meshmeshing.cpp
		 )
		

set(INCS ${EIGEN_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})
set(LIBS trimesh2 clipper)
set(DEFS CGAL_NO_GMP=1 CGAL_NO_MPFR=1 CGAL_EIGEN3_ENABLED CGAL_HEADER_ONLY)

__add_platform_library(cmesh)

if(CC_INSTALL_CMESH)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/ cmesh)
endif()
