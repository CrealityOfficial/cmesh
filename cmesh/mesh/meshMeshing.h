#ifndef MESH_MESHING_1605318972342_H
#define MESH_MESHING_1605318972342_H
//#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

//#include <CGAL/Polyhedron_3.h>
//#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
//typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
//typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;

namespace cmesh
{
	//void refineFair(trimesh::TriMesh* mesh, Polyhedron* poly);
	//void isTriangulate(trimesh::TriMesh* mesh);
	void remeshIsotropic(trimesh::TriMesh* mesh);

	void holeFilling(trimesh::TriMesh* mesh);
}

#endif // MESH_MESHING_1605318972342_H