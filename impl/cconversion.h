#ifndef CONVERSION_1605318972342_H
#define CONVERSION_1605318972342_H
#include "ctype.h"
#include "trimesh2/TriMesh.h"

namespace cmesh
{
	void _convertT2C(const trimesh::TriMesh& tmesh, CMesh& mesh);
	void _convertC2T(const CMesh& mesh, trimesh::TriMesh& tmesh);
}
#endif // CONVERSION_1605318972342_H