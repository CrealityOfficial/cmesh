__cc_find(TriMesh2)
__assert_target(trimesh2)

__cc_find(Boost)
__cc_find(Eigen)
__cc_find(CGAL)
__assert_target(cgal)
__assert_target(eigen)
__cc_find(tbb)
__assert_target(boost_system)

if (MSVC)
  add_definitions(/bigobj)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /bigobj")
endif ()

add_definitions( -DNOMINMAX -DCGAL_LINKED_WITH_TBB)

set(SRCS include/cmesh/interface.h
		 include/cmesh/poly/roof.h
		 include/cmesh/mesh/boolean.h
		 include/cmesh/mesh/richmesh.h
		 include/cmesh/mesh/repair.h
                	 include/cmesh/mesh/pedestal.h
		 include/cmesh/cgal/subrepairmenu.h
		 include/cmesh/cgal/subrepairtype.h
        		 include/cmesh/uvs/adduvs.h
		 
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
		 impl/tmeshutil.h
		 impl/tmeshutil.cpp
		 impl/repairNew.h
		 impl/repairNew.cpp
		 impl/pedestal.cpp
		 impl/cgal/subrepair.h
		 impl/cgal/subrepair.cpp
		 impl/cgal/subrepairmenu.cpp
		 impl/adduvs.cpp
		 )
		 
if(USE_IGL)
		list(APPEND SRCS 
			include/cmesh/igl/booleanigl.h
			impl/igl/booleanigl.cpp
			)
endif()

		

set(INCS ${CMAKE_CURRENT_SOURCE_DIR}/include/)
set(INTERFACES ${CMAKE_CURRENT_SOURCE_DIR}/include/)
set(LIBS trimesh2 clipper boost_system mmesh tbb cgal eigen)
set(DEFS CGAL_EIGEN3_ENABLED CGAL_HEADER_ONLY __TBBMALLOC_NO_IMPLICIT_LINKAGE=1)

if(USE_IGL)
	list(APPEND INCS ${BOOST_INCLUDE_DIRS} ${mpfr_INCLUDE_DIRS})	
	list(APPEND DEFS IGL_STATIC_LIBRARY CGAL_USE_CORE=1)	
	list(APPEND LIBS igl copyleft mpfr gmp)	
endif()

__add_common_library(cmesh)

add_definitions(-DCGAL_NO_GMP=0)
add_definitions(-DCGAL_NO_MPFR=0)

if(CC_INSTALL_CMESH)
	__install_directory_specif(${CMAKE_CURRENT_SOURCE_DIR}/include/ cmesh)
endif()
