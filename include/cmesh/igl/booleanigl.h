#ifndef MMESH_BOOLEANIGL_1607481576423_H
#define MMESH_BOOLEANIGL_1607481576423_H
#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"
#include "ccglobal/tracer.h"

namespace cmesh
{

    CMESH_API trimesh::TriMesh* cxBooleanOperateMeshIGLObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2, ccglobal::Tracer* tracer = nullptr);

    CMESH_API trimesh::TriMesh* cxBooleanOperateMeshIGLObj(std::string& Mesh1Name, std::string& Mesh2Name, ccglobal::Tracer* tracer = nullptr);
}
#endif // MMESH_BOOLEANIGL_1607481576423_H