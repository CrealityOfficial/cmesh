#include "meshwrapper.h"
#include "trimesh2/Vec.h"
#include "meshgeneration.h"
#include "meshmeshing.h"

#include <CGAL/Polyhedron_3.h>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;


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

	trimesh::TriMesh* MeshWrapper::fillHolesWrapper()
	{
		selfIntersections(m_mesh);

		
		deleteOutlier(m_mesh);

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

		holeFilling(m_mesh);
		//fillHoles(m_mesh, (Polyhedron*)m_impl);

		return m_mesh;
	}
}