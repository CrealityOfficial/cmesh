#include "subrepair.h"
#include "../cconversion.h"

namespace cmesh
{
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

    void CGALisotropicRemeshing(CMesh& cmesh, double target_edge_length, ccglobal::Tracer* trace)
    {
        if (!CGAL::is_triangle_mesh(cmesh))
        {
            return ;
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
    }

    void CGALcorefinement(CMesh& cmesh1, CMesh& cmesh2, ccglobal::Tracer* trace)
    {
        if (!PMP::does_self_intersect(cmesh1)
            && !PMP::does_self_intersect(cmesh2))
        {
            PMP::corefine(cmesh1, cmesh2);
        }
    }

    void CGALtriangulate(CMesh& cmesh1, ccglobal::Tracer* trace)
    {
        PMP::triangulate_faces(cmesh1);
        // Confirm that all faces are triangles.
        for (boost::graph_traits<CMesh>::face_descriptor f : faces(cmesh1))
            if (!CGAL::is_triangle(halfedge(f, cmesh1), cmesh1))
                std::cerr << "Error: non-triangular face left in mesh." << std::endl;
    }

    void CGALboolean(CMesh& cmesh1, CMesh& cmesh2, CMesh& cmeshOut, CGALBooleanType type, ccglobal::Tracer* trace)
    {
        if (!PMP::does_self_intersect(cmesh1)
            && !PMP::does_self_intersect(cmesh2)
            && PMP::does_bound_a_volume(cmesh1)
            && PMP::does_bound_a_volume(cmesh2))
        {
            switch (type)
            {
            case cmesh::CGALBooleanType::CGAL_UNION:
                PMP::corefine_and_compute_union(cmesh1, cmesh2, cmeshOut);
                break;
            case cmesh::CGALBooleanType::CGAL_INTERSECTION:
                PMP::corefine_and_compute_intersection(cmesh1, cmesh2, cmeshOut);
                break;
            case cmesh::CGALBooleanType::CGAL_DIFFERENCE:
                PMP::corefine_and_compute_difference(cmesh1, cmesh2, cmeshOut);
                break;
            default:
                break;
            }
            
        }
    }

    void CGALholeFill(CMesh& cmesh, CGALHoleFillType type, ccglobal::Tracer* trace)
    {
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
    }

    bool CGALselfIntersections(CMesh& cmesh, std::vector<int>& faceIndexs, bool dealFace,ccglobal::Tracer* trace)
    {
        bool intersecting = PMP::does_self_intersect<CGAL::Parallel_if_available_tag>(cmesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, cmesh)));
        if (intersecting)
        {
            std::vector<std::pair<face_descriptor, face_descriptor> > intersected_tris;
            PMP::self_intersections<CGAL::Parallel_if_available_tag>(faces(cmesh), cmesh, std::back_inserter(intersected_tris));
       
            for (int i = 0; i < intersected_tris.size(); ++i)
            {
                faceIndexs.push_back(intersected_tris[i].first);
                faceIndexs.push_back(intersected_tris[i].second);
            }
            std::sort(faceIndexs.begin(), faceIndexs.end());
            faceIndexs.erase(std::unique(faceIndexs.begin(), faceIndexs.end()), faceIndexs.end());

            if (dealFace)
            {
                CMesh cmeshnew;
                for (Point v : cmesh.points())
                {
                    cmeshnew.add_vertex(v);
                }
                for (CMesh::Face_index face_index : cmesh.faces())
                {
                    CGAL::Vertex_around_face_circulator<CMesh> vcirc(cmesh.halfedge(face_index), cmesh), done(vcirc);

                    if (faceIndexs.end() == std::find(faceIndexs.begin(), faceIndexs.end(), face_index))
                    {
                        int f[3];
                        int index = 0;
                        do
                        {
                            f[index] = (*vcirc).idx();
                            if (f[index] < 0)
                            {
                                break;
                            }
                            index += 1;
                        } while (++vcirc != done && index < 3);

                        cmeshnew.add_face(CGAL::SM_Vertex_index(f[0]), CGAL::SM_Vertex_index(f[1]), CGAL::SM_Vertex_index(f[2]));
                    }
                }
                cmesh = cmeshnew;
                cmeshnew.clear();
                return true;
            }
        }
        return false;
    }

    void CGALorientation(CMesh& cmesh, bool reversible, ccglobal::Tracer* trace)
    {
        if (CGAL::is_closed(cmesh))
        {
            CGAL::Polygon_mesh_processing::orient(cmesh);
        }

        //if (!CGAL::Polygon_mesh_processing::is_outward_oriented(cmesh))
        //{
        //    CGAL::Polygon_mesh_processing::reverse_face_orientations(cmesh);
        //}

        if (CGAL::is_closed(cmesh))
            CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(cmesh);

        if (reversible)
        {
            CGAL::Polygon_mesh_processing::reverse_face_orientations(cmesh);
        }
    }

    void CGALstitch(CMesh& cmesh, ccglobal::Tracer* trace)
    {
        int unm = PMP::stitch_borders(cmesh);
    }

    void CGALmanifoldness(CMesh& cmesh, ccglobal::Tracer* trace)
    { 
        std::vector<std::vector<vertex_descriptor> > duplicated_vertices;
        std::size_t new_vertices_nb = PMP::duplicate_non_manifold_vertices(cmesh,
            NP::output_iterator(
                std::back_inserter(duplicated_vertices)));
    }

    void CGALconnectedComponents(CMesh& cmesh, ccglobal::Tracer* trace)
    {
        //typedef boost::graph_traits<CMesh>::face_descriptor face_descriptor;
        //const double bound = std::cos(0.75 * CGAL_PI);
        //std::vector<face_descriptor> cc;
        //face_descriptor fd = *faces(cmesh).first;

        ////PMP::connected_component(fd,
        ////    cmesh,
        ////    boost::make_function_output_iterator(Put_true<F_select_map>(fselect_map)));
    }

    void CGALdetectFeatures(CMesh& cmesh, ccglobal::Tracer* trace)
    {

        typedef boost::property_map<CMesh, CGAL::edge_is_feature_t>::type EIFMap;
        typedef boost::property_map<CMesh, CGAL::face_patch_id_t<int> >::type PIMap;
        typedef boost::property_map<CMesh, CGAL::vertex_incident_patches_t<int> >::type VIMap;
        EIFMap eif = get(CGAL::edge_is_feature, cmesh);
        PIMap pid = get(CGAL::face_patch_id_t<int>(), cmesh);
        VIMap vip = get(CGAL::vertex_incident_patches_t<int>(), cmesh);
        std::size_t number_of_patches
            = PMP::sharp_edges_segmentation(cmesh, 10, eif, pid,
                CGAL::parameters::vertex_incident_patches_map(vip));
        std::size_t nb_sharp_edges = 0;
        for (boost::graph_traits<CMesh>::edge_descriptor e : edges(cmesh))
        {
            if (get(eif, e))
                ++nb_sharp_edges;
        }

        std::size_t nb_sharp_faces= 0;
        for (boost::graph_traits<CMesh>::face_descriptor f : faces(cmesh))
        {
            if (get(pid, f))
                ++nb_sharp_faces;
        }

        std::cout << "This mesh contains " << nb_sharp_edges << " sharp edges" << std::endl;
        std::cout << " and " << number_of_patches << " surface patches." << std::endl;
    }
}