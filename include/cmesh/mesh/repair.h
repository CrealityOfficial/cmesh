#ifndef CMESH_REPAIR_1648026566432_H
#define CMESH_REPAIR_1648026566432_H
#include "cmesh/mesh/richmesh.h"
#include "ccglobal/tracer.h"

namespace cmesh
{
	struct ErrorInfo
	{
		int edgeNum;
		int normalNum;
	};

	CMESH_API void getErrorInfo(const RichMesh& mesh, ErrorInfo& info);

	struct HoleFillParam
	{

	};

	CMESH_API void repairHole(RichMesh& mesh, const HoleFillParam& param, ccglobal::Tracer* tracer);
}

#endif // CMESH_REPAIR_1648026566432_H