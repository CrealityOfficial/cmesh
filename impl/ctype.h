#ifndef CMESH_CGAL_TYPE_1605318972342_H
#define CMESH_CGAL_TYPE_1605318972342_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/corefinement.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Polygon_mesh_processing/remesh.h>
#include <CGAL/Polygon_mesh_processing/triangulate_hole.h>

#include <CGAL/Polygon_mesh_processing/corefinement.h>

#include <CGAL/Polygon_mesh_processing/repair.h>

#include <CGAL/Polygon_mesh_processing/orientation.h>

#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>

#include <CGAL/boost/graph/iterator.h>

#include <CGAL/Polygon_mesh_processing/triangulate_faces.h>

#include <CGAL/Polygon_mesh_processing/connected_components.h>

//
#include <CGAL/Polygon_mesh_processing/orientation.h>
#include <CGAL/Polygon_mesh_processing/polygon_soup_to_polygon_mesh.h>
#include <CGAL/Polygon_mesh_processing/orient_polygon_soup_extension.h>

#include <CGAL/Polygon_mesh_processing/self_intersections.h>

#include <CGAL/Polygon_mesh_processing/detect_features.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel   K;
typedef CGAL::Exact_predicates_inexact_constructions_kernel   Kernel;
typedef Kernel::Point_3                                       Point;
typedef CGAL::Surface_mesh<K::Point_3>                        CMesh;
typedef boost::graph_traits<CMesh>::halfedge_descriptor        halfedge_descriptor;
typedef boost::graph_traits<CMesh>::edge_descriptor            edge_descriptor;
typedef boost::graph_traits<CMesh>::vertex_descriptor        vertex_descriptor;
typedef boost::graph_traits<CMesh>::halfedge_descriptor      halfedge_descriptor;
typedef boost::graph_traits<CMesh>::face_descriptor          face_descriptor;

namespace PMP = CGAL::Polygon_mesh_processing;
namespace params = CGAL::Polygon_mesh_processing::parameters;
namespace NP = CGAL::parameters;

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

#endif // CMESH_CGAL_TYPE_1605318972342_H