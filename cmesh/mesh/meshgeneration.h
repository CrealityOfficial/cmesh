#ifndef MESH_GENERATION_1605318972342_H
#define MESH_GENERATION_1605318972342_H
//#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;

namespace cmesh
{
	void selfIntersections(trimesh::TriMesh* mesh);
	void deleteOutlier(trimesh::TriMesh* mesh);
	trimesh::TriMesh* fillHoles(trimesh::TriMesh* mesh , Polyhedron* poly);
	void mesh2polyhedron(trimesh::TriMesh* mesh, Polyhedron* P);
}

#endif // MESH_GENERATION_1605318972342_H