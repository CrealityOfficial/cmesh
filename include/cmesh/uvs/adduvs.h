#ifndef ADDUVS_1622032440408_H
#define ADDUVS_1622032440408_H
#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

namespace cmesh
{

    CMESH_API void addUVs(trimesh::TriMesh* mesh,ccglobal::Tracer* tracer = nullptr);
}

#endif // ADDUVS_1622032440408_H