#ifndef TMESHUTIL_1605318972342_H
#define TMESHUTIL_1605318972342_H
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

void spiltModel(trimesh::TriMesh* mesh, std::vector<trimesh::TriMesh*>& validMeshes, ccglobal::Tracer* tracer);


#endif // TMESHUTIL_1605318972342_H