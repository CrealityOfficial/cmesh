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

    void _convertT2CForNoRepair(trimesh::TriMesh& tmesh, CMesh& mesh, bool bDeleteSamallFace)
    {
        mesh.clear();
        std::vector<vertex_descriptor> vertexdescriptor;
        vertexdescriptor.reserve(tmesh.vertices.size());
        for (const trimesh::point& p : tmesh.vertices) {
            vertexdescriptor.push_back(mesh.add_vertex(Point(p.x, p.y, p.z)));
        }
        if (bDeleteSamallFace)
        {
            float angle = 180.0f;
            for (int i = 0; i < tmesh.faces.size(); i++) {
                const trimesh::TriMesh::Face& f = tmesh.faces[i];
                if (f.x != f.y && f.x != f.z && f.y != f.z)
                {
                    angle = 180.0f;
                    //mmesh::GetArea(tmesh.vertices[f.x], tmesh.vertices[f.y], tmesh.vertices[f.z]) > 0.0f)
                    for (size_t j = 0; j < 3; j++)
                    {
                        trimesh::point& p0 = tmesh.vertices[f[j]];
                        trimesh::point& p1 = tmesh.vertices[f[(j + 1) % 3]];
                        trimesh::point& p2 = tmesh.vertices[f[(j + 2) % 3]];
                        trimesh::point np0p1 = trimesh::normalized(p1 - p0);
                        trimesh::point np0p2 = trimesh::normalized(p2 - p0);

                        angle = std::min(angle, acos(np0p1 DOT np0p2));
                    }
                    if (angle > 0.001)//3бу
                    {
                        mesh.add_face(vertexdescriptor[f.x], vertexdescriptor[f.y], vertexdescriptor[f.z]);
                    }
                }
            }
        }
        else
        {
            for (const trimesh::TriMesh::Face& f : tmesh.faces) {
                if (f.x != f.y && f.x != f.z && f.y != f.z)
                {

                    mesh.add_face(vertexdescriptor[f.x], vertexdescriptor[f.y], vertexdescriptor[f.z]);
                }
            }
        }
    }

    
    bool _convertT2CCGAL(trimesh::TriMesh& tmesh, CMesh& mesh, bool needRepair)
    {
        std::vector<Point> points;
        std::vector<std::vector<std::size_t> > faces;
        for (const trimesh::point& p : tmesh.vertices) {
            points.push_back(Point(p.x, p.y, p.z));
        }
        for (const trimesh::TriMesh::Face& f : tmesh.faces) {
            std::vector<std::size_t> vec;
            vec.push_back(f.x);
            vec.push_back(f.y);
            vec.push_back(f.z);
            faces.push_back(vec);
        }

        //if (!CGAL::IO::read_polygon_soup(fname, points, faces))
        //{
        //    if (verbose)
        //        std::cerr << "Warning: cannot read polygon soup" << std::endl;
        //    return false;
        //}

        if (needRepair)
            PMP::repair_polygon_soup(points, faces);

        if (!PMP::orient_polygon_soup(points, faces))
        {
            //if (verbose)
            //    std::cerr << "Some duplication happened during polygon soup orientation" << std::endl;
        }

        if (!PMP::is_polygon_soup_a_polygon_mesh(faces))
        {
            //if (verbose)
                //std::cerr << "Warning: polygon soup does not describe a polygon mesh" << std::endl;
            return false;
        }

        PMP::polygon_soup_to_polygon_mesh(points, faces, mesh);

        return true;
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

    void _convertTs2T(CMesh& mesh, std::vector< CMesh>& meshs)
    {
        for (size_t i = 0; i < meshs.size(); i++)
        {
            std::vector<vertex_descriptor> vertexdescriptor;
            for (const Point& v : meshs[i].points()) {
                vertexdescriptor.push_back(mesh.add_vertex(v));
            }

            for (CMesh::Face_index face_index : meshs[i].faces())
            {
                trimesh::TriMesh::Face f;
                // Mesh::Vertex_around_face_circulator fvit(surfaceMesh.halfedge(face_index), surfaceMesh);
                CGAL::Vertex_around_face_circulator<CMesh> vcirc(meshs[i].halfedge(face_index), meshs[i]), done(vcirc);
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
                }
                mesh.add_face(vertexdescriptor[f.x], vertexdescriptor[f.y], vertexdescriptor[f.z]);
            }
        }
    }
}