#ifndef REPAIRNEW_1605318972342_H
#define REPAIRNEW_1605318972342_H
#include "ctype.h"
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

namespace cmesh
{
	trimesh::TriMesh* repairMenuNew(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer);
	trimesh::TriMesh* generatePedestal(trimesh::TriMesh* inMesh, ccglobal::Tracer* tracer);

	void removeNorVector2(trimesh::TriMesh* mesh);
	void splitTmesh2Cmesh2(trimesh::TriMesh* mesh, std::vector<CMesh>& outMeshes, ccglobal::Tracer* tracer);
	void selfIntersections2(CMesh& cmesh);
}
#endif // REPAIRNEW_1605318972342_H