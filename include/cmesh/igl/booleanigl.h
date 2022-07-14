#ifndef MMESH_BOOLEANIGL_1607481576423_H
#define MMESH_BOOLEANIGL_1607481576423_H
#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"
#include "ccglobal/tracer.h"

namespace cmesh
{

    enum MeshBooleanType
    {
        MESH_BOOLEAN_TYPE_UNION = 0,
        MESH_BOOLEAN_TYPE_INTERSECT = 1,
        MESH_BOOLEAN_TYPE_MINUS = 2,
        MESH_BOOLEAN_TYPE_XOR = 3,
        MESH_BOOLEAN_TYPE_RESOLVE = 4,
        NUM_MESH_BOOLEAN_TYPES = 5
    };

    CMESH_API trimesh::TriMesh* cxBooleanOperateMeshIGLObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2, MeshBooleanType meshBooleanType,ccglobal::Tracer* tracer = nullptr);

    CMESH_API trimesh::TriMesh* cxBooleanOperateMeshIGLObj(std::string& Mesh1Name, std::string& Mesh2Name, MeshBooleanType meshBooleanType, ccglobal::Tracer* tracer = nullptr);
}
#endif // MMESH_BOOLEANIGL_1607481576423_H