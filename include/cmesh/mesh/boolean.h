#ifndef MMESH_BOOLEAN_1607481576423_H
#define MMESH_BOOLEAN_1607481576423_H
#include "cmesh/interface.h"
#include "cmesh/mesh/richmesh.h"

namespace cmesh
{
    enum class CMeshBooleanType
    {
        CBT_UNION,
        CBT_INTERSECTION,
        CBT_TM1_MINUS_TM2,
        CBT_TM2_MINUS_TM1,
    };

    CMESH_API trimesh::TriMesh* cxBooleanOperateMeshObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2, CMeshBooleanType type);
    CMESH_API RichMesh* boolean(RichMesh& mesh1, RichMesh& mesh2, CMeshBooleanType type);
}
#endif // MMESH_BOOLEAN_1607481576423_H