#ifndef CMESH_REPAIR_1648026566432_H
#define CMESH_REPAIR_1648026566432_H
#include "cmesh/mesh/richmesh.h"
#include "ccglobal/tracer.h"

namespace cmesh
{
	struct ErrorInfo
	{
		int edgeNum;      //ȱ�ݱ�
		int normalNum;    //ȱ�ݵķ���
		int holeNum;      //�׶�����
		int intersectNum; //������
	};

	CMESH_API void getErrorInfo(RichMesh& mesh, ErrorInfo& info);

	CMESH_API void getErrorInfo(trimesh::TriMesh* mesh, ErrorInfo& info);

	//typedef enum 
	//{
	//	refile_hole,
	//	refine_and_fair_hole
	//} HoleFillParam;

	//CMESH_API void repairHole(RichMesh& mesh, const HoleFillParam& param, ccglobal::Tracer* tracer);



	CMESH_API void repairMenu(RichMesh& mesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer);

	CMESH_API void repairMenu(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer);
}

#endif // CMESH_REPAIR_1648026566432_H