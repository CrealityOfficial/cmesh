#ifndef MMESH_BOOLEANIGL_1607481576423_H
#define MMESH_BOOLEANIGL_1607481576423_H
#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

namespace cmesh
{

    CMESH_API trimesh::TriMesh* cxBooleanOperateMeshIGLObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2);

    CMESH_API trimesh::TriMesh* cxBooleanOperateMeshIGLObj(std::string& Mesh1Name, std::string& Mesh2Name);
}
#endif // MMESH_BOOLEANIGL_1607481576423_H