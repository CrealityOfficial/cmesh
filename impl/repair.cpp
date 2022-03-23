#include "cmesh/mesh/repair.h"
#include "cholefill.h"

namespace cmesh
{
	void getErrorInfo(const RichMesh& mesh, ErrorInfo& info)
	{

	}

	void repairHole(RichMesh& mesh, const HoleFillParam& param, ccglobal::Tracer* tracer)
	{
		_holeFilling(mesh.impl().mesh, tracer);
	}
}