#include "cmesh/cgal/subrepair.h"
#include "../cconversion.h"
#include "../ctype.h"

namespace cmesh
{
    struct halfedge2edge
    {
        halfedge2edge(const CMesh& m, std::vector<edge_descriptor>& edges)
            : m_mesh(m), m_edges(edges)
        {}
        void operator()(const halfedge_descriptor& h) const
        {
            m_edges.push_back(edge(h, m_mesh));
        }
        const CMesh& m_mesh;
        std::vector<edge_descriptor>& m_edges;
    };

    

    // Optional visitor for orientating a polygon soup to demonstrate usage for some functions.
    // inherits from the default class as some functions are not overloaded
    struct Visitor : public CGAL::Polygon_mesh_processing::Default_orientation_visitor
    {
        void non_manifold_edge(std::size_t id1, std::size_t id2, std::size_t nb_poly)
        {
            std::cout << "The edge " << id1 << ", " << id2 << " is not manifold: " << nb_poly << " incident polygons." << std::endl;
        }
        void non_manifold_vertex(std::size_t id, std::size_t nb_cycles)
        {
            std::cout << "The vertex " << id << " is not manifold: " << nb_cycles << " connected components of vertices in the link." << std::endl;
        }
        void duplicated_vertex(std::size_t v1, std::size_t v2)
        {
            std::cout << "The vertex " << v1 << " has been duplicated, its new id is " << v2 << "." << std::endl;
        }
        void vertex_id_in_polygon_replaced(std::size_t p_id, std::size_t i1, std::size_t i2)
        {
            std::cout << "In the polygon " << p_id << ", the index " << i1 << " has been replaced by " << i2 << "." << std::endl;
        }
        void polygon_orientation_reversed(std::size_t p_id)
        {
            std::cout << "The polygon " << p_id << " has been reversed." << std::endl;
        }
    };

    trimesh::TriMesh* isotropicRemeshing(trimesh::TriMesh* mesh, double target_edge_length, ccglobal::Tracer* trace)
    {
        CMesh cmesh;
        _convertT2CForNoRepair(*mesh, cmesh);

        if (!CGAL::is_triangle_mesh(cmesh))
        {
            return nullptr;
        }

        //double target_edge_length = 0.04;
        unsigned int nb_iter = 3;
        std::cout << "Split border...";
        std::vector<edge_descriptor> border;
        PMP::border_halfedges(faces(cmesh), cmesh, boost::make_function_output_iterator(halfedge2edge(cmesh, border)));
        PMP::split_long_edges(border, target_edge_length, cmesh);

        //PMP::isotropic_remeshing(faces(cmesh), target_edge_length, cmesh,
        //    CGAL::parameters::number_of_iterations(nb_iter)
        //    .protect_constraints(true)); //i.e. protect border, here

        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh, *newMesh);
        return newMesh;
    }

    trimesh::TriMesh* corefinement(trimesh::TriMesh* mesh1, trimesh::TriMesh* mesh2, ccglobal::Tracer* trace)
    {
        CMesh cmesh1;
        CMesh cmesh2;
        _convertT2CForNoRepair(*mesh1, cmesh1);
        _convertT2CForNoRepair(*mesh2, cmesh2);

        if (!PMP::does_self_intersect(cmesh1)
            && !PMP::does_self_intersect(cmesh2))
        {
            PMP::corefine(cmesh1, cmesh2);
        }
            

        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh1, *newMesh);
        return newMesh;
    }

    trimesh::TriMesh* boolean(trimesh::TriMesh* mesh1, trimesh::TriMesh* mesh2, CGALBooleanType type, ccglobal::Tracer* trace)
    {
        CMesh cmesh1;
        CMesh cmesh2;
        _convertT2CForNoRepair(*mesh1, cmesh1);
        _convertT2CForNoRepair(*mesh2, cmesh2);

        CMesh cmesh3;
        if (!PMP::does_self_intersect(cmesh1)
            && !PMP::does_self_intersect(cmesh2)
            && PMP::does_bound_a_volume(cmesh1)
            && PMP::does_bound_a_volume(cmesh2))
        {
            switch (type)
            {
            case cmesh::CGALBooleanType::CGAL_UNION:
                PMP::corefine_and_compute_union(cmesh1, cmesh2, cmesh3);
                break;
            case cmesh::CGALBooleanType::CGAL_INTERSECTION:
                PMP::corefine_and_compute_intersection(cmesh1, cmesh2, cmesh3);
                break;
            case cmesh::CGALBooleanType::CGAL_DIFFERENCE:
                PMP::corefine_and_compute_difference(cmesh1, cmesh2, cmesh3);
                break;
            default:
                break;
            }
            
        }

        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh3, *newMesh);
        return newMesh;
    }

    trimesh::TriMesh* holeFill(trimesh::TriMesh* mesh, CGALHoleFillType type, ccglobal::Tracer* trace)
    {
        CMesh cmesh;
        _convertT2CForNoRepair(*mesh, cmesh);

        std::vector<halfedge_descriptor> border_cycles;
        // collect one halfedge per boundary cycle
        PMP::extract_boundary_cycles(cmesh, std::back_inserter(border_cycles));

        for (halfedge_descriptor h : border_cycles)
        {
            std::vector<face_descriptor>  patch_facets;
            std::vector<vertex_descriptor> patch_vertices;
            switch (type)
            {
            case cmesh::CGALHoleFillType::CGAL_TRIANGULATE:
                PMP::triangulate_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets));
                break;
            case cmesh::CGALHoleFillType::CGAL_REFINED:
                PMP::triangulate_and_refine_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices));
                break;
            case cmesh::CGALHoleFillType::CGAL_FAIRED:
                std::get<0>(PMP::triangulate_refine_and_fair_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices)));
                break;
            default:
                break;
            }
        }

        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh, *newMesh);
        return newMesh;
    }

    void selfIntersections(trimesh::TriMesh* mesh, std::vector<int>& faceIndex,ccglobal::Tracer* trace)
    {
        CMesh cmesh;
        _convertT2CForNoRepair(*mesh, cmesh);

        bool intersecting = PMP::does_self_intersect<CGAL::Parallel_if_available_tag>(cmesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, cmesh)));
        if (intersecting)
        {
            std::vector<std::pair<face_descriptor, face_descriptor> > intersected_tris;
            PMP::self_intersections<CGAL::Parallel_if_available_tag>(faces(cmesh), cmesh, std::back_inserter(intersected_tris));
       
            for (int i = 0; i < intersected_tris.size(); ++i)
            {
                faceIndex.push_back(intersected_tris[i].first);
                faceIndex.push_back(intersected_tris[i].second);
            }
            std::sort(faceIndex.begin(), faceIndex.end());
            faceIndex.erase(std::unique(faceIndex.begin(), faceIndex.end()), faceIndex.end());
        }
    }

    trimesh::TriMesh* orientation(trimesh::TriMesh* mesh, bool reversible, ccglobal::Tracer* trace)
    {
        CMesh cmesh;
        _convertT2CForNoRepair(*mesh, cmesh);

        if (CGAL::is_closed(cmesh))
            CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(cmesh);

        if (reversible)
        {
            CGAL::Polygon_mesh_processing::reverse_face_orientations(cmesh);
        }

        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh, *newMesh);
        return newMesh;
    }

    trimesh::TriMesh* stitch(trimesh::TriMesh* mesh, ccglobal::Tracer* trace)
    {
        CMesh cmesh;
        _convertT2CForNoRepair(*mesh, cmesh);

        int unm = PMP::stitch_borders(cmesh);

        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh, *newMesh);
        return newMesh;
    }

    trimesh::TriMesh* manifoldness(trimesh::TriMesh* mesh, ccglobal::Tracer* trace)
    { 
        CMesh cmesh;
        _convertT2CForNoRepair(*mesh, cmesh);

        std::vector<std::vector<vertex_descriptor> > duplicated_vertices;
        std::size_t new_vertices_nb = PMP::duplicate_non_manifold_vertices(cmesh,
            NP::output_iterator(
                std::back_inserter(duplicated_vertices)));

        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh, *newMesh);
        return newMesh;
    }

    trimesh::TriMesh* connectedComponents(trimesh::TriMesh* mesh, ccglobal::Tracer* trace)
    {
        //CMesh cmesh;
        //_convertT2CForNoRepair(*mesh, cmesh);

        //typedef boost::graph_traits<CMesh>::face_descriptor face_descriptor;
        //const double bound = std::cos(0.75 * CGAL_PI);
        //std::vector<face_descriptor> cc;
        //face_descriptor fd = *faces(cmesh).first;

        ////PMP::connected_component(fd,
        ////    cmesh,
        ////    boost::make_function_output_iterator(Put_true<F_select_map>(fselect_map)));

        //trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        //_convertC2T(cmesh, *newMesh);
        //return newMesh;

        return nullptr;
    }
}