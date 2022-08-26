#include "subrepair.h"
#include "../cconversion.h"

#include <CGAL/Polygon_mesh_processing/compute_normal.h>


typedef boost::graph_traits<CMesh>::vertex_descriptor      vertex_descriptor;
typedef boost::graph_traits<CMesh>::face_descriptor        face_descriptor;
typedef K::Vector_3                                               Vector;

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

    void CGALsmoothing(CMesh& cmesh, double angle, ccglobal::Tracer* trace)
    {
        // Constrain edges with a dihedral angle over 60°
        typedef boost::property_map<CMesh, CGAL::edge_is_feature_t>::type EIFMap;
        EIFMap eif = get(CGAL::edge_is_feature, cmesh);
        PMP::detect_sharp_edges(cmesh, angle, eif);
        int sharp_counter = 0;
        for (edge_descriptor e : edges(cmesh))
            if (get(eif, e))
                ++sharp_counter;
        std::cout << sharp_counter << " sharp edges" << std::endl;
        const unsigned int nb_iterations = 10;
        std::cout << "Smoothing mesh... (" << nb_iterations << " iterations)" << std::endl;
        // Smooth with both angle and area criteria + Delaunay flips
        PMP::angle_and_area_smoothing(cmesh, CGAL::parameters::number_of_iterations(nb_iterations)
            .use_safety_constraints(false) // authorize all moves
            .edge_is_constrained_map(eif));

    }

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

    //三角化:修复自交问题
    void CGALtangential(CMesh& cmesh, ccglobal::Tracer* trace)
    {
        unsigned int nb_iter = 10;
        std::cout << "Relax...";
        PMP::tangential_relaxation(cmesh, CGAL::parameters::number_of_iterations(nb_iter));
        std::cout << "done." << std::endl;
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
                    std::back_inserter(patch_facets),
                    CGAL::parameters::use_delaunay_triangulation(true));
                break;
            case cmesh::CGALHoleFillType::CGAL_REFINED:
                PMP::triangulate_and_refine_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices),
                    CGAL::parameters::use_delaunay_triangulation(true));
                break;
            case cmesh::CGALHoleFillType::CGAL_FAIRED://CGAL::Parallel_if_available_tag
                std::get<0>(PMP::triangulate_refine_and_fair_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices),
                    CGAL::parameters::use_delaunay_triangulation(true)));
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
       

            if (intersected_tris.size())
            {
                faceIndexs.push_back(intersected_tris[0].first);
            }
            for (int i = 1; i < intersected_tris.size(); ++i)
            {
                if (faceIndexs.end() == std::find(faceIndexs.begin(), faceIndexs.end(),intersected_tris[i].first)
                    && faceIndexs.end() == std::find(faceIndexs.begin(), faceIndexs.end(), intersected_tris[i].second))
                {
                    faceIndexs.push_back(intersected_tris[i].first);
                    //faceIndexs.push_back(intersected_tris[i].second);
                }
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
        if (!CGAL::is_closed(cmesh))
            return;

        {
            //CGAL::Polygon_mesh_processing::orient(cmesh);
        }

        auto vnormals = cmesh.add_property_map<vertex_descriptor, Vector>("v:normals", CGAL::NULL_VECTOR).first;
        auto fnormals = cmesh.add_property_map<face_descriptor, Vector>("f:normals", CGAL::NULL_VECTOR).first;
        PMP::compute_normals(cmesh, vnormals, fnormals);

        ////if (!CGAL::Polygon_mesh_processing::is_outward_oriented(cmesh))
        ////{
        ////    CGAL::Polygon_mesh_processing::reverse_face_orientations(cmesh);
        ////}

        ////if (CGAL::is_closed(cmesh))
        ////    CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(cmesh);

        //if (reversible)
        //{
        //    CGAL::Polygon_mesh_processing::reverse_face_orientations(cmesh);
        //}
        std::vector<CMesh> cmeshs;
        CGAL::Polygon_mesh_processing::split_connected_components(cmesh,cmeshs);
        for (size_t i = 0; i < cmeshs.size(); i++)
        {
            if (!CGAL::is_valid_polygon_mesh(cmeshs[i]))
                continue;

            try {
                if (CGAL::is_closed(cmeshs[i]))
                {
                    CGAL::Polygon_mesh_processing::orient(cmeshs[i]);
                }

                //if (!CGAL::Polygon_mesh_processing::is_outward_oriented(cmesh))
                //{
                //    CGAL::Polygon_mesh_processing::reverse_face_orientations(cmesh);
                //}

                if (CGAL::is_closed(cmeshs[i]))
                    CGAL::Polygon_mesh_processing::orient_to_bound_a_volume(cmeshs[i]);


                if (reversible)
                {
                    CGAL::Polygon_mesh_processing::reverse_face_orientations(cmeshs[i]);
                }
            }
            catch (std::exception & e)
            {
                return;
            }

        }
        cmesh.clear();
        _convertTs2T(cmesh, cmeshs);
       
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

        const double bound = std::cos(0.75 * CGAL_PI);
        PMP::keep_large_connected_components(cmesh,
            2,
            CGAL::parameters::edge_is_constrained_map(Constraint<CMesh>(cmesh, bound)));
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
            = PMP::sharp_edges_segmentation(cmesh, 90, eif, pid,
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