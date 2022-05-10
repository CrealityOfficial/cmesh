#include "cconversion.h"
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

namespace cmesh
{
	void _convertT2C(trimesh::TriMesh& tmesh, CMesh& mesh, bool needRepair)
	{
		std::vector<Point> points;
		std::vector<std::vector<std::size_t> > faces;
		for (unsigned i = 0; i < tmesh.vertices.size(); i++) {
			points.push_back(Point(tmesh.vertices[i].x, tmesh.vertices[i].y, tmesh.vertices[i].z));
		}

		for (unsigned i = 0; i < tmesh.faces.size(); i++) {
			std::vector<std::size_t> v;
			v.push_back(tmesh.faces[i].x);
			v.push_back(tmesh.faces[i].y);
			v.push_back(tmesh.faces[i].z);
			faces.push_back(v);
		}
		if (needRepair)
			PMP::repair_polygon_soup(points, faces, CGAL::parameters::verbose(true));

		PMP::polygon_soup_to_polygon_mesh(points, faces, mesh);
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