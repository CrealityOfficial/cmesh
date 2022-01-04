#ifndef MMESH_BOOLEAN_1607481576423_H
#define MMESH_BOOLEAN_1607481576423_H
#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

namespace cmesh
{
    CMESH_API enum  class cxBoolean_operation_type
    {
        CX_UNION,
        CX_INTERSECTION,
        CX_TM1_MINUS_TM2,
        CX_TM2_MINUS_TM1,
    };

    CMESH_API trimesh::TriMesh* cxBooleanOperateMeshObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2, cxBoolean_operation_type typeindex);
}
#endif // MMESH_BOOLEAN_1607481576423_H