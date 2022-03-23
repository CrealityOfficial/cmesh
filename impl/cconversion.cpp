#include "cconversion.h"

namespace cmesh
{
	void _convertT2C(const trimesh::TriMesh& tmesh, CMesh& mesh)
	{
		for (unsigned i = 0; i < tmesh.vertices.size(); i++) {
			mesh.add_vertex(Point(tmesh.vertices[i].x, tmesh.vertices[i].y, tmesh.vertices[i].z));
		}
		for (unsigned i = 0; i < tmesh.faces.size(); i++) {
			const int& f1 = tmesh.faces[i].x;
			const int& f2 = tmesh.faces[i].y;
			const int& f3 = tmesh.faces[i].z;
			if (f1 != f2 && f1 != f3 && f2 != f3)
			{
				mesh.add_face(CGAL::SM_Vertex_index(tmesh.faces[i].x), CGAL::SM_Vertex_index(tmesh.faces[i].y), CGAL::SM_Vertex_index(tmesh.faces[i].z));
			}
		}

		//CGAL::Polygon_mesh_processing::remove_isolated_vertices(mesh;
	}

	void _convertC2T(const CMesh& mesh, trimesh::TriMesh& tmesh)
	{
		tmesh.vertices.clear();
		tmesh.faces.clear();
		for (Point v : mesh.points())
		{
			tmesh.vertices.push_back(trimesh::point(v.x(), v.y(), v.z()));
		}
		for (CMesh::Face_index face_index : mesh.faces())
		{
			trimesh::TriMesh::Face f;
			// Mesh::Vertex_around_face_circulator fvit(surfaceMesh.halfedge(face_index), surfaceMesh);
			CGAL::Vertex_around_face_circulator<CMesh> vcirc(mesh.halfedge(face_index), mesh), done(vcirc);
			int index = 0;
			if (vcirc)
			{
				do
				{
					//std::cout << (*vcirc).idx() << std::endl;
					f[index] = (*vcirc).idx();
					if (f[index] < 0)
					{
						break;
					}
					index += 1;
				} while (++vcirc != done && index < 3);
				if (index == 3)
				{
					if (f.x != f.y && f.x != f.z && f.y != f.z)
					{
						tmesh.faces.emplace_back(f);
					}
				}
			}
		}
	}
}