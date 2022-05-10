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
}
#endif // REPAIRNEW_1605318972342_H