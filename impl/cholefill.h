#ifndef HOLEFILL_1605318972342_H
#define HOLEFILL_1605318972342_H
#include "richmeshimpl.h"
#include "ccglobal/tracer.h"
#include "trimesh2/TriMesh.h"

namespace cmesh
{
	bool _holeFilling(CMesh& cmesh, bool refine_and_fair_hole,ccglobal::Tracer* tracer);

	bool _holeFilling(CMesh& cmesh, std::vector<CMesh>& cmeshs , bool refine_and_fair_hole, ccglobal::Tracer* tracer);

	bool pedestalFilling(CMesh& cmesh, float fZ,bool isSmooth,ccglobal::Tracer* tracer);

	bool directionDetection(CMesh& cmesh, trimesh::TriMesh* atrimesh, ccglobal::Tracer* tracer);
}


#endif // HOLEFILL_1605318972342_H