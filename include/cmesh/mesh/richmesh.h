#ifndef CMESH_RICHMESH_1648024266468_H
#define CMESH_RICHMESH_1648024266468_H
#include "cmesh/interface.h"
#include "trimesh2/TriMesh.h"

namespace cmesh
{
	class RichMeshImpl;
	class CMESH_API RichMesh
	{
	public:
		RichMesh();
		~RichMesh();

		void initFromTriMesh(trimesh::TriMesh* mesh);
		trimesh::TriMesh* generateTrimesh();

		RichMeshImpl& impl();
	protected:
		RichMeshImpl* m_impl;
	};
}

#endif // CMESH_RICHMESH_1648024266468_H