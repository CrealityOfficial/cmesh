#include "meshwrapper.h"
#include "trimesh2/Vec.h"
#include "meshgeneration.h"
#include "meshmeshing.h"

#include "ccglobal/tracer.h"

namespace cmesh
{
	MeshWrapper::MeshWrapper()
	{
		
	}

	MeshWrapper::~MeshWrapper()
	{
		//if (m_impl)
		//{
		//	delete m_impl;
		//	m_impl = nullptr;
		//}
	}

	void MeshWrapper::initMesh(trimesh::TriMesh* mesh)
	{
		m_mesh = mesh;
	}

	trimesh::TriMesh* MeshWrapper::fillHolesWrapper(ccglobal::Tracer* tracer)
	{
		if (tracer)
		{
			tracer->progress(0.2f);
		}
		selfIntersections(m_mesh);

		if (tracer)
		{
			tracer->progress(0.4f);
		}
		//deleteOutlier(m_mesh);

		//remeshIsotropic(m_mesh);

		//if (!m_impl)
		//{
		//	delete m_impl;
		//	m_impl = nullptr;
		//}
		//Polyhedron* p = new Polyhedron();
		//mesh2polyhedron(m_mesh, p);
		//m_impl = (void*)p;

		//refineFair(m_mesh, (Polyhedron*)m_impl);
		//isTriangulate(m_mesh);

		holeFilling(m_mesh, tracer);
		//fillHoles(m_mesh, (Polyhedron*)m_impl);

		if (tracer)
		{
			tracer->progress(1.0f);
		}
		return m_mesh;
	}
}