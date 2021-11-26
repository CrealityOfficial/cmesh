__assert_target(trimesh2)

if(NOT EIGEN_INCLUDE_DIRS)
	message(FATAL_ERROR "Please Specified EIGEN_INCLUDE_DIRS")
endif()

if(NOT BOOST_INCLUDE_DIRS)
	message(FATAL_ERROR "Please Specified BOOST_INCLUDE_DIRS")
endif()

include(CGAL)

__cgal_include_package(Straight_skeleton_2)
__cgal_include_package(Polygon)
__cgal_include_package(Polygon_mesh_processing)
__cgal_include_package(Polyhedron)
__cgal_include_package(Polyhedron_IO)
__cgal_include_package(Random_numbers)
__cgal_include_package(Box_intersection_d) 
__cgal_include_package(AABB_tree)
__cgal_include_package(Spatial_searching)
__cgal_include_package(Spatial_sorting)
__cgal_include_package(Generator)
__cgal_include_package(Union_find)
__cgal_include_package(Modifier)
__cgal_include_package(Triangulation_2)
__cgal_include_package(TDS_2)
__cgal_include_package(Surface_mesh)
__cgal_include_package(Surface_mesh_simplification)

#enable cluster point 
__cgal_include_package(Point_set_processing_3)
__cgal_include_package(Point_set_3)
__cgal_include_package(Solver_interface)
__cgal_include_package(Triangulation_3)
__cgal_include_package(TDS_3)
__cgal_include_package(Subdivision_method_3)

set(SRCS cmesh/interface.h
		 cmesh/mesh/simplify.h
		 cmesh/mesh/simplify.cpp
		 cmesh/mesh/roof.h
		 cmesh/mesh/roof.cpp
		 )
		
set(INCS ${EIGEN_INCLUDE_DIRS} ${BOOST_INCLUDE_DIRS})
set(LIBS trimesh2)
set(DEFS CGAL_NO_GMP=1 CGAL_NO_MPFR=1 CGAL_EIGEN3_ENABLED)

if(BUILD_CMESH_STATIC)
	__add_real_target(cmesh lib SOURCE ${SRCS}
								  LIB ${LIBS}
								  DEF ${DEFS}
								  INC ${INCS}
								  INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
								  SOURCE_FOLDER
								  )
else()
	list(APPEND DEFS CMESH_DLL)
	__add_real_target(cmesh dll SOURCE ${SRCS}
								  LIB ${LIBS}
								  DEF ${DEFS}
								  INC ${INCS}
								  INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}
								  SOURCE_FOLDER
								  )
endif()

