#ifndef CONVERSION_1605318972342_H
#define CONVERSION_1605318972342_H
#include "ctype.h"
#include "trimesh2/TriMesh.h"

namespace cmesh
{
	void _convertT2C(trimesh::TriMesh& tmesh, CMesh& mesh, bool needRepair = false);
    void _convertT2CForNoRepair(trimesh::TriMesh& tmesh, CMesh& mesh, bool bDeleteSamallFace = false);
	void _convertC2T(const CMesh& mesh, trimesh::TriMesh& tmesh);
    void _convertTs2T(CMesh& mesh, std::vector< CMesh>& meshs);


    bool _convertT2CCGAL(trimesh::TriMesh& tmesh, CMesh& mesh, bool needRepair = false);
}
#endif // CONVERSION_1605318972342_H