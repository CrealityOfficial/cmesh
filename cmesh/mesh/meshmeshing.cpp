#include "meshmeshing.h"
#include "trimesh2/Vec.h"

#if 0
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/refine.h>
#include <CGAL/Polygon_mesh_processing/fair.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>                          Polyhedron;
typedef Polyhedron::Vertex_handle                           Vertex_handle;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;

namespace PMP = CGAL::Polygon_mesh_processing;

namespace cmesh
{
    void extract_k_ring(Vertex_handle v,
        int k,
        std::vector<Vertex_handle>& qv)
    {
        std::map<Vertex_handle, int>  D;
        qv.push_back(v);
        D[v] = 0;
        std::size_t current_index = 0;
        int dist_v;
        while (current_index < qv.size() && (dist_v = D[qv[current_index]]) < k)
        {
            v = qv[current_index++];
            Polyhedron::Halfedge_around_vertex_circulator e(v->vertex_begin()), e_end(e);
            do {
                Vertex_handle new_v = e->opposite()->vertex();
                if (D.insert(std::make_pair(new_v, dist_v + 1)).second)
                    qv.push_back(new_v);
            } while (++e != e_end);
        }
    }

	void refineFair(trimesh::TriMesh* mesh, Polyhedron* poly)
	{
        std::vector<Polyhedron::Facet_handle>  new_facets;
        std::vector<Vertex_handle> new_vertices;
        PMP::refine(*poly, faces(*poly),
            std::back_inserter(new_facets),
            std::back_inserter(new_vertices),
            CGAL::parameters::density_control_factor(2.));

        mesh->vertices.clear();
        mesh->faces.clear();
        mesh->normals.clear();

        //vertices(*ploy);

        for(Vertex_handle v:vertices(*poly))
        {
            mesh->vertices.push_back(trimesh::point(v->point().x(), v->point().y(), v->point().z()));
        }

        for(Polyhedron::Facet_handle h :faces(*poly))
        {
            Halfedge_facet_circulator j = h->facet_begin();
            // Facets in polyhedral surfaces are at least triangles.
            if (CGAL::circulator_size(j) < 3)
                continue;
            trimesh::TriMesh::Face f;

            f.x = distance(poly->vertices_begin(), j->vertex());
            f.y = distance(poly->vertices_begin(), j->next()->vertex());
            f.z = distance(poly->vertices_begin(), j->next()->next()->vertex());

            mesh->faces.push_back(f);
        }


        //Polyhedron::Vertex_iterator v = poly->vertices_begin();
        //std::advance(v, 82/*e.g.*/);
        //std::vector<Vertex_handle> region;
        //extract_k_ring(v, 12/*e.g.*/, region);
        //bool success = PMP::fair(poly, region);
        //std::cout << "Fairing : " << (success ? "succeeded" : "failed") << std::endl;
        //std::ofstream faired_off("faired.off");
        //faired_off.precision(17);
        //faired_off << poly;
        //faired_off.close();
	}
}

#endif

#if 0
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                     Point;
typedef CGAL::Surface_mesh<Point>                           Surface_mesh;
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
namespace PMP = CGAL::Polygon_mesh_processing;

namespace cmesh
{
    void isTriangulate(trimesh::TriMesh* mesh)
    {
        Surface_mesh smesh;
        for (unsigned i = 0; i < mesh->vertices.size(); i++) {
            smesh.add_vertex(K::Point_3(mesh->vertices[i].x, mesh->vertices[i].y, mesh->vertices[i].z));
        }
        for (unsigned i = 0; i < mesh->faces.size(); i++) {
            smesh.add_face(CGAL::SM_Vertex_index(mesh->faces[i].x), CGAL::SM_Vertex_index(mesh->faces[i].y), CGAL::SM_Vertex_index(mesh->faces[i].z));
        }


        PMP::triangulate_faces(smesh);
        // Confirm that all faces are triangles.
        for (boost::graph_traits<Surface_mesh>::face_descriptor f : faces(smesh))
            if (!CGAL::is_triangle(halfedge(f, smesh), smesh))
                int a = 0;
    }

}

#endif

#if 0
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <boost/iterator/function_output_iterator.hpp>
typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef CGAL::Surface_mesh<K::Point_3>                        Mesh;
//typedef boost::graph_traits<Mesh>::halfedge_descriptor        halfedge_descriptor;
typedef boost::graph_traits<Mesh>::edge_descriptor            edge_descriptor;
typedef K::Point_3                                      Point;
namespace PMP = CGAL::Polygon_mesh_processing;

namespace cmesh
{
    struct halfedge2edge
    {
        halfedge2edge(const Mesh& m, std::vector<edge_descriptor>& edges)
            : m_mesh(m), m_edges(edges)
        {}
        void operator()(const halfedge_descriptor& h) const
        {
            m_edges.push_back(edge(h, m_mesh));
        }
        const Mesh& m_mesh;
        std::vector<edge_descriptor>& m_edges;
    };

    void remeshIsotropic(trimesh::TriMesh* mesh)
    {
        Mesh smesh;
        for (unsigned i = 0; i < mesh->vertices.size(); i++) {
            smesh.add_vertex(K::Point_3(mesh->vertices[i].x, mesh->vertices[i].y, mesh->vertices[i].z));
        }
        for (unsigned i = 0; i < mesh->faces.size(); i++) {
            smesh.add_face(CGAL::SM_Vertex_index(mesh->faces[i].x), CGAL::SM_Vertex_index(mesh->faces[i].y), CGAL::SM_Vertex_index(mesh->faces[i].z));
        }

        double target_edge_length = 0.04; // 0.04;
        unsigned int nb_iter = 3;
        std::cout << "Split border...";
        std::vector<edge_descriptor> border;
        PMP::border_halfedges(faces(smesh), smesh, boost::make_function_output_iterator(halfedge2edge(smesh, border)));
        PMP::split_long_edges(border, target_edge_length, smesh);

        int a = num_faces(smesh);
        PMP::isotropic_remeshing(faces(smesh), target_edge_length, smesh,
            PMP::parameters::number_of_iterations(nb_iter)
            .protect_constraints(true)); //i.e. protect border, here
        std::cout << "Remeshing done." << std::endl;

        mesh->vertices.clear();
        mesh->faces.clear();
        mesh->normals.clear();

        //vertices(*ploy);

        //Vertex_range vertices()
        for (Point v : smesh.points())
        {
            mesh->vertices.push_back(trimesh::point(v.x(), v.y(), v.z()));
        }
        for (Mesh::Face_index face_index : smesh.faces())
        {
            trimesh::TriMesh::Face f;
            // Mesh::Vertex_around_face_circulator fvit(surfaceMesh.halfedge(face_index), surfaceMesh);
            CGAL::Vertex_around_face_circulator<Mesh> vcirc(smesh.halfedge(face_index), smesh), done(vcirc);
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
                    mesh->faces.emplace_back(f);
            }
        }

        //for (CGAL::SM_Vertex_index v : vertices(smesh))
        //{
        //    v.idx
        //    mesh->vertices.push_back(trimesh::point(v->point().x(), v->point().y(), v->point().z()));
        //}

        //for (Polyhedron::Facet_handle h : faces(smesh))
        //{
        //    Halfedge_facet_circulator j = h->facet_begin();
        //    // Facets in polyhedral surfaces are at least triangles.
        //    if (CGAL::circulator_size(j) < 3)
        //        continue;
        //    trimesh::TriMesh::Face f;

        //    f.x = distance(smesh.vertices_begin(), j->vertex());
        //    f.y = distance(smesh.vertices_begin(), j->next()->vertex());
        //    f.z = distance(smesh.vertices_begin(), j->next()->next()->vertex());

        //    mesh->faces.push_back(f);
        //}
    }
}
#endif


#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>

#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <boost/lexical_cast.hpp>

#include <iostream>
#include <fstream>
#include <vector>
#include <set>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef Kernel::Point_3                                     Point;
typedef CGAL::Surface_mesh<Point>                           Mesh;
typedef boost::graph_traits<Mesh>::vertex_descriptor        vertex_descriptor;
typedef boost::graph_traits<Mesh>::halfedge_descriptor      halfedge_descriptor;
typedef boost::graph_traits<Mesh>::face_descriptor          face_descriptor;
namespace PMP = CGAL::Polygon_mesh_processing;

namespace cmesh {
    bool is_small_hole(halfedge_descriptor h, Mesh& mesh,
        double max_hole_diam, int max_num_hole_edges)
    {
        int num_hole_edges = 0;
        CGAL::Bbox_3 hole_bbox;
        for (halfedge_descriptor hc : CGAL::halfedges_around_face(h, mesh))
        {
            const Point& p = mesh.point(target(hc, mesh));
            hole_bbox += p.bbox();
            ++num_hole_edges;
            // Exit early, to avoid unnecessary traversal of large holes
            if (num_hole_edges > max_num_hole_edges) return false;
            if (hole_bbox.xmax() - hole_bbox.xmin() > max_hole_diam) return false;
            if (hole_bbox.ymax() - hole_bbox.ymin() > max_hole_diam) return false;
            if (hole_bbox.zmax() - hole_bbox.zmin() > max_hole_diam) return false;
        }
        return true;
    }
    void holeFilling(trimesh::TriMesh* mesh)
    {
        Mesh smesh;
        for (unsigned i = 0; i < mesh->vertices.size(); i++) {
            smesh.add_vertex(Point(mesh->vertices[i].x, mesh->vertices[i].y, mesh->vertices[i].z));
        }
        for (unsigned i = 0; i < mesh->faces.size(); i++) {
            smesh.add_face(CGAL::SM_Vertex_index(mesh->faces[i].x), CGAL::SM_Vertex_index(mesh->faces[i].y), CGAL::SM_Vertex_index(mesh->faces[i].z));
        }

        // Both of these must be positive in order to be considered
        double max_hole_diam = -1.0;
        int max_num_hole_edges = -1;

        unsigned int nb_holes = 0;
        std::vector<halfedge_descriptor> border_cycles;
        // collect one halfedge per boundary cycle
        //PMP::extract_boundary_cycles(smesh, std::back_inserter(border_cycles));
        for (halfedge_descriptor h : border_cycles)
        {
            if (max_hole_diam > 0 && max_num_hole_edges > 0 &&
                !is_small_hole(h, smesh, max_hole_diam, max_num_hole_edges))
                continue;
            std::vector<face_descriptor>  patch_facets;
            std::vector<vertex_descriptor> patch_vertices;
            bool success = std::get<0>(PMP::triangulate_refine_and_fair_hole(smesh,
                h,
                std::back_inserter(patch_facets),
                std::back_inserter(patch_vertices)));
            std::cout << "* Number of facets in constructed patch: " << patch_facets.size() << std::endl;
            std::cout << "  Number of vertices in constructed patch: " << patch_vertices.size() << std::endl;
            std::cout << "  Is fairing successful: " << success << std::endl;
            ++nb_holes;
        }

        mesh->vertices.clear();
        mesh->faces.clear();
        mesh->normals.clear();

        for (Point v : smesh.points())
        {
            mesh->vertices.push_back(trimesh::point(v.x(), v.y(), v.z()));
        }
        for (Mesh::Face_index face_index : smesh.faces())
        {
            trimesh::TriMesh::Face f;
            // Mesh::Vertex_around_face_circulator fvit(surfaceMesh.halfedge(face_index), surfaceMesh);
            CGAL::Vertex_around_face_circulator<Mesh> vcirc(smesh.halfedge(face_index), smesh), done(vcirc);
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
                    mesh->faces.emplace_back(f);
            }
        }

    }
}
