#include "cmesh/igl/booleanigl.h"
#include <Eigen/Core>
#include <igl/copyleft/cgal/mesh_boolean.h>

namespace cmesh
{
	typedef struct Emesh{
		Eigen::MatrixXd V;
		Eigen::MatrixXi F;
	}emesh;

	void T2E(trimesh::TriMesh* Mesh1, emesh& emesh)
	{
		if (!Mesh1)
		{
			return;
		}
		// allocate space for vertices
		const int number_of_vertices = Mesh1->vertices.size();
		emesh.V.resize(number_of_vertices, 3);
		for (int i = 0; i < number_of_vertices; i++)
		{
			const trimesh::point& p = Mesh1->vertices[i];
			emesh.V(i, 0) = p.x;
			emesh.V(i, 1) = p.y;
			emesh.V(i, 2) = p.z;
		}

		int number_of_triangles = Mesh1->faces.size();
		// allocate space for triangles
		emesh.F.resize(number_of_triangles, 3);
		for (int i = 0; i < number_of_triangles; i++)
		{
			trimesh::TriMesh::Face& f = Mesh1->faces[i];
			emesh.F(i, 0) = f.x;
			emesh.F(i, 1) = f.y;
			emesh.F(i, 2) = f.z;
		}
	}

	void E2T(trimesh::TriMesh* Mesh1, emesh& emesh)
	{
		Mesh1 = new trimesh::TriMesh();

		const int number_of_vertices = emesh.V.size();
		Mesh1->vertices.reserve(number_of_vertices);
		for (int i = 0; i < number_of_vertices; i++)
		{
			Mesh1->vertices.push_back(trimesh::point(emesh.V(i, 0), emesh.V(i, 1), emesh.V(i, 2)));
		}

		int number_of_triangles = emesh.F.size();
		// allocate space for triangles
		Mesh1->faces.reserve(number_of_triangles);
		for (int i = 0; i < number_of_triangles; i++)
		{
			trimesh::TriMesh::Face f ;
			f.x = emesh.F(i, 0);
			f.y = emesh.F(i, 1);
			f.z = emesh.F(i, 2);
		}
	}

	trimesh::TriMesh* cxBooleanOperateMeshIGLObj(trimesh::TriMesh* Mesh1, trimesh::TriMesh* Mesh2)
	{

		emesh e1, e2, e0;

		T2E(Mesh1, e1);
		T2E(Mesh1, e2);

		igl::copyleft::cgal::mesh_boolean(e1.V, e1.F, e2.V, e2.F,
			igl::MESH_BOOLEAN_TYPE_MINUS,
			e0.V, e0.F);

		trimesh::TriMesh* out;
		out->need_normals();
		return out;
	}
}





