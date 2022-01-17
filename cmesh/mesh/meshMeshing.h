#ifndef MESH_MESHING_1605318972342_H
#define MESH_MESHING_1605318972342_H
//#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

namespace cmesh
{
	//void refineFair(trimesh::TriMesh* mesh, Polyhedron* poly);
	//void isTriangulate(trimesh::TriMesh* mesh);
	void remeshIsotropic(trimesh::TriMesh* mesh);

	void holeFilling(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer);
}

#endif // MESH_MESHING_1605318972342_H