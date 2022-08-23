#ifndef CMESH_PEDESTAL_1648026566432_H
#define CMESH_PEDESTAL_1648026566432_H
#include "cmesh/mesh/richmesh.h"
#include "ccglobal/tracer.h"

namespace cmesh
{
	enum class Direction
	{
		automatic,
		uper,
		down
	};
	struct PedestalParam
	{
		float height;
		bool isSmooth;
		Direction adirection;
	};


	CMESH_API trimesh::TriMesh* pedestalMenu(trimesh::TriMesh* mesh, PedestalParam apram, ccglobal::Tracer* tracer);
}

#endif // CMESH_PEDESTAL_1648026566432_H