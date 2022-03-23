#include "cmesh/mesh/richmesh.h"
#include "richmeshimpl.h"
#include "cconversion.h"

namespace cmesh
{
	RichMesh::RichMesh()
	{
		m_impl = new RichMeshImpl();
	}

	RichMesh::~RichMesh()
	{
		delete m_impl;
	}

	RichMeshImpl& RichMesh::impl()
	{
		return *m_impl;
	}

	void RichMesh::initFromTriMesh(trimesh::TriMesh* mesh)
	{
		if (!mesh)
			return;

		m_impl->mesh.clear();
		_convertT2C(*mesh, m_impl->mesh);
	}

	trimesh::TriMesh* RichMesh::generateTrimesh()
	{
		trimesh::TriMesh* mesh = new trimesh::TriMesh();

		_convertC2T(m_impl->mesh, *mesh);
		if (mesh->vertices.size() == 0 && mesh->faces.size() == 0)
		{
			delete mesh;
			mesh = nullptr;
		}
		return mesh;
	}
}