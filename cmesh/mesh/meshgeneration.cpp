#include "meshgeneration.h"
#include "trimesh2/Vec.h"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>
#include <boost/foreach.hpp>
//Intersections
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/self_intersections.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel Kernel;
typedef CGAL::Polyhedron_3<Kernel>     Polyhedron;
typedef Polyhedron::Halfedge_handle    Halfedge_handle;
typedef Polyhedron::Face               Facet;
typedef Polyhedron::Facet_iterator     Facet_iterator;
typedef Polyhedron::Point_iterator     Point_iterator;
typedef Polyhedron::Facet_handle       Facet_handle;
typedef Polyhedron::Vertex_handle      Vertex_handle;
typedef Polyhedron::Halfedge_around_facet_circulator Halfedge_facet_circulator;
//Intersections
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Surface_mesh<K::Point_3>                      CMesh;
typedef boost::graph_traits<CMesh>::face_descriptor          face_descriptor;
namespace PMP = CGAL::Polygon_mesh_processing;

namespace cmesh
{
    // A modifier converting a Maya mesh to CGAL Polyhedron_3
    template <class HDS>
    class Mesh_to_polyhedron : public CGAL::Modifier_base<HDS> {
    public:
        Mesh_to_polyhedron(trimesh::TriMesh* mesh) : m_mesh(mesh) {}
        void operator()(HDS& hds) {
            // get mesh data
            //MFloatPointArray pts;
            //m_mesh.getPoints(pts);
            //MIntArray tcounts, tvers;
            //m_mesh.getTriangles(tcounts, tvers);

            // Postcondition: `hds' is a valid polyhedral surface.
            CGAL::Polyhedron_incremental_builder_3<HDS> B(hds, true);
            B.begin_surface(m_mesh->vertices.size(), m_mesh->faces.size());
            // vertices
            typedef typename HDS::Vertex::Point Vertex;
            typedef typename Vertex Point;
            for (unsigned i = 0; i < m_mesh->vertices.size(); i++) {
                HDS::Vertex_handle vh = B.add_vertex(Point(m_mesh->vertices[i].x, m_mesh->vertices[i].y, m_mesh->vertices[i].z));
                //vh->id = i;
            }

            // triangles
            for (unsigned i = 0; i < m_mesh->faces.size(); i++) {
                HDS::Face_handle fh = B.begin_facet();
                B.add_vertex_to_facet(m_mesh->faces[i].x);
                B.add_vertex_to_facet(m_mesh->faces[i].y);
                B.add_vertex_to_facet(m_mesh->faces[i].z);
                B.end_facet();
                //fh->id = i;
            }

            B.end_surface();
        }

    private:
        trimesh::TriMesh* m_mesh;
    };

    void mesh2polyhedron(trimesh::TriMesh* mesh, Polyhedron* P)
    {
        Mesh_to_polyhedron<Polyhedron::HalfedgeDS> builder(mesh);
        P->delegate(builder);
    }

    trimesh::TriMesh* fillHoles(trimesh::TriMesh* mesh, Polyhedron* poly)
    {
        int num = num_faces(*poly);

        // Incrementally fill the holes
        BOOST_FOREACH(Halfedge_handle h, halfedges(*poly))
        {
            if (h->is_border())
            {
                std::vector<Facet_handle>  patch_facets;
                std::vector<Vertex_handle> patch_vertices;
                bool success = CGAL::cpp11::get<0>(
                    CGAL::Polygon_mesh_processing::triangulate_refine_and_fair_hole(
                        *poly,
                        h,
                        std::back_inserter(patch_facets),
                        std::back_inserter(patch_vertices),
                        CGAL::Polygon_mesh_processing::parameters::vertex_point_map(get(CGAL::vertex_point, *poly)).
                        geom_traits(Kernel())));

                for (size_t i = 0; i < patch_vertices.size(); i++)
                {
                    mesh->vertices.push_back(trimesh::point(patch_vertices[i]->point().x(), patch_vertices[i]->point().y(), patch_vertices[i]->point().z()));
                }
                for (size_t i = 0; i < patch_facets.size(); i++)
                {
                    Halfedge_facet_circulator j = patch_facets[i]->facet_begin();
                    // Facets in polyhedral surfaces are at least triangles.
                    if (CGAL::circulator_size(j) < 3)
                        continue;
                    trimesh::TriMesh::Face f;
                    f.x = distance(poly->vertices_begin(), j->vertex());
                    f.y = distance(poly->vertices_begin(), j->next()->vertex());
                    f.z = distance(poly->vertices_begin(), j->next()->next()->vertex());

                    mesh->faces.push_back(f);
                }
            }
        }

        return mesh;
    }

    void selfIntersections(trimesh::TriMesh* mesh)
    {
        CMesh cgalMesh;
        for (unsigned i = 0; i < mesh->vertices.size(); i++) {
            cgalMesh.add_vertex(K::Point_3(mesh->vertices[i].x, mesh->vertices[i].y, mesh->vertices[i].z));
            //vh->id = i;
        }

        // triangles
        for (unsigned i = 0; i < mesh->faces.size(); i++) {
            cgalMesh.add_face(CGAL::SM_Vertex_index(mesh->faces[i].x), CGAL::SM_Vertex_index(mesh->faces[i].y), CGAL::SM_Vertex_index(mesh->faces[i].z));
            //fh->id = i;
        }

        bool intersecting = PMP::does_self_intersect(cgalMesh,
            PMP::parameters::vertex_point_map(get(CGAL::vertex_point, cgalMesh)));

        std::vector<std::pair<face_descriptor, face_descriptor> > intersected_tris;
        //PMP::self_intersections<CGAL::Parallel_if_available_tag>(faces(cgalMesh), cgalMesh, std::back_inserter(intersected_tris));
        int num = intersected_tris.size();

        std::vector<int>delFace;
        for (int i = 0; i < intersected_tris.size(); ++i)
        {
            delFace.push_back(intersected_tris[i].first);
            delFace.push_back(intersected_tris[i].second);
        }
        std::sort(delFace.begin(), delFace.end());
        delFace.erase(std::unique(delFace.begin(), delFace.end()), delFace.end());

//        std::vector<int> nerbors;
//        nerbors.reserve(delFace.size()*3);
//        if (mesh->across_edge.size() == mesh->faces.size())
//        {
//#if defined(_OPENMP)
//#pragma omp parallel for
//#endif
//            for (size_t i = 0; i < delFace.size(); i++)
//            {
//                if (mesh->across_edge[delFace[i]][0]>0)
//                    nerbors.push_back(mesh->across_edge[delFace[i]][0]);
//                if (mesh->across_edge[delFace[i]][1] > 0)
//                    nerbors.push_back(mesh->across_edge[delFace[i]][1]);
//                if (mesh->across_edge[delFace[i]][2] > 0)
//                    nerbors.push_back(mesh->across_edge[delFace[i]][2]);
//            }
//        }
//        delFace.insert(delFace.end(), nerbors.begin(), nerbors.end());
//
//        std::sort(delFace.begin(), delFace.end());
//        delFace.erase(std::unique(delFace.begin(), delFace.end()), delFace.end());

        if (delFace.size()>0)
        {
            std::vector<trimesh::TriMesh::Face> validFaces;
            for (size_t i = 0; i < mesh->faces.size(); i++)
            {
                if (!count(delFace.begin(), delFace.end(), i))
                    validFaces.push_back(mesh->faces.at(i));
            }
            mesh->faces.swap(validFaces);
        }
    }


    void deleteOutlier(trimesh::TriMesh* mesh)
    {
        mesh->across_edge.clear();
        mesh->adjacentfaces.clear();
        mesh->need_across_edge();
        int nf = mesh->faces.size();
        std::vector<trimesh::TriMesh::Face> validFaces;

        for (int i = 0; i < nf; i++) {
            if (mesh->across_edge[i][0] != -1 || mesh->across_edge[i][1] != -1|| mesh->across_edge[i][2] != -1)  
            {
               if(mesh->across_edge[i][0] != mesh->across_edge[i][1]
                   && mesh->across_edge[i][1] != mesh->across_edge[i][2]
                   && mesh->across_edge[i][0] != mesh->across_edge[i][2])
                validFaces.push_back(mesh->faces.at(i));
            }
        }
        mesh->faces.swap(validFaces);
    }
}