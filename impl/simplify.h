#ifndef MMESH_SIMPLIFY_1607481576423_H
#define MMESH_SIMPLIFY_1607481576423_H
#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

namespace trimesh
{
    class TriMesh;
}

namespace cmesh
{
    CMESH_API trimesh::TriMesh* cxSimplifyOperateMeshObj(trimesh::TriMesh* meshObj);
}

#endif // MMESH_SIMPLIFY_1607481576423_H