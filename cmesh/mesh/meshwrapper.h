#ifndef MESH_WRAPPER_1605318972342_H
#define MESH_WRAPPER_1605318972342_H
#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

namespace ccglobal
{
	class Tracer;
}

namespace cmesh
{
	class CMESH_API MeshWrapper
	{
	public:
		MeshWrapper();
		~MeshWrapper();
		void initMesh(trimesh::TriMesh* mesh);
		trimesh::TriMesh* fillHolesWrapper(ccglobal::Tracer* tracer);
	protected:
		trimesh::TriMesh* m_mesh;
		void* m_impl;
	};
}

#endif // MESH_WRAPPER_1605318972342_H