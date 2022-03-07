#include "cmesh/poly/roof.h"

#include <boost/shared_ptr.hpp>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/create_straight_skeleton_from_polygon_with_holes_2.h>

typedef CGAL::Exact_predicates_inexact_constructions_kernel K;

typedef K::Point_2                    Point;
typedef CGAL::Polygon_2<K>            Polygon_2;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;
typedef CGAL::Straight_skeleton_2<K>  Straight_skeleton;
typedef Straight_skeleton::Halfedge_const_iterator Halfedge_const_iterator;
typedef Straight_skeleton::Halfedge_const_handle   Halfedge_const_handle;
typedef Straight_skeleton::Vertex_const_iterator Vertex_const_iterator;
typedef Straight_skeleton::Vertex_const_handle Vertex_const_handle;
typedef Straight_skeleton::Face_const_iterator Face_const_iterator;
typedef Straight_skeleton::Face_const_handle Face_const_handle;

typedef boost::shared_ptr<Straight_skeleton> Straight_skeleton_ptr;

namespace cmesh
{
    void seperate1423(ClipperLibXYZ::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs)
    {
        for (ClipperLibXYZ::PolyNode* node1 : polyTree->Childs)
        {
            std::vector<ClipperLibXYZ::PolyNode*>& node2 = node1->Childs;
            std::vector<ClipperLibXYZ::PolyNode*> node3;
            for (ClipperLibXYZ::PolyNode* n : node2)
                node3.insert(node3.end(), n->Childs.begin(), n->Childs.end());
            std::vector<ClipperLibXYZ::PolyNode*> node4;
            for (ClipperLibXYZ::PolyNode* n : node3)
                node4.insert(node4.end(), n->Childs.begin(), n->Childs.end());

            PolyPair* pair1 = new PolyPair();
            pair1->clockwise = false;
            pair1->outer = node1;
            pair1->inner.swap(node4);
            polyPairs.push_back(pair1);

            for (ClipperLibXYZ::PolyNode* n : node2)
            {
                PolyPair* pair = new PolyPair();
                pair->clockwise = true;
                pair->outer = n;
                pair->inner = n->Childs;
                polyPairs.push_back(pair);
            }
        }
    }

    void seperate1234(ClipperLibXYZ::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs)
    {
        for (ClipperLibXYZ::PolyNode* node1 : polyTree->Childs)
        {
            std::vector<ClipperLibXYZ::PolyNode*>& node2 = node1->Childs;
            std::vector<ClipperLibXYZ::PolyNode*> node3;
            for (ClipperLibXYZ::PolyNode* n : node2)
                node3.insert(node3.end(), n->Childs.begin(), n->Childs.end());
            std::vector<ClipperLibXYZ::PolyNode*> node4;
            for (ClipperLibXYZ::PolyNode* n : node3)
                node4.insert(node4.end(), n->Childs.begin(), n->Childs.end());

            PolyPair* pair1 = new PolyPair();
            pair1->clockwise = false;
            pair1->outer = node1;
            pair1->inner.swap(node2);
            polyPairs.push_back(pair1);

            for (ClipperLibXYZ::PolyNode* n : node3)
            {
                PolyPair* pair = new PolyPair();
                pair->clockwise = true;
                pair->outer = n;
                pair->inner = n->Childs;
                polyPairs.push_back(pair);
            }
        }
    }

    void fill_polygon(Polygon_2& polygon, ClipperLibXYZ::Path* path, bool inverse)
    {
        size_t size = path->size();
        if (!inverse)
        {
            for (size_t i = 0; i < size; ++i)
            {
                ClipperLibXYZ::IntPoint& p = path->at(i);
                polygon.push_back(Point(INT2MM(p.X), INT2MM(p.Y)));
            }
        }
        else
        {
            for (size_t i = 0; i < size; ++i)
            {
                ClipperLibXYZ::IntPoint& p = path->at(size - 1 - i);
                polygon.push_back(Point(INT2MM(p.X), INT2MM(p.Y)));
            }
        }
    }

    void build_polygon_with_holes(Polygon_with_holes* input, PolyPair* pair)
    {
        bool inverse = pair->clockwise;

        Polygon_2& poly = input->outer_boundary();
        ClipperLibXYZ::Path& path = pair->outer->Contour;
        size_t size = path.size();
        if (size < 3 /*|| inverse*/)
            return;

        fill_polygon(input->outer_boundary(), &path, inverse);

        for (ClipperLibXYZ::PolyNode* n : pair->inner)
        {
            ClipperLibXYZ::Path& npath = n->Contour;
            size_t nsize = npath.size();
            if (nsize < 3)
                return;

            Polygon_2 hole;
            fill_polygon(hole, &npath, inverse);

            input->add_hole(hole);
        }
    }

    bool test_simple_polygon(Polygon_with_holes& input)
    {
        //check the validity of the input and fix orientation
        if (!input.outer_boundary().is_simple())
        {
            std::cerr << "ERROR: outer boundary is not simple.";
            return false;
        }

        bool valid = true;
        for (Polygon_with_holes::Hole_iterator it = input.holes_begin();
            it != input.holes_end(); ++it)
        {
            if (!it->is_simple())
            {
                valid = false;
                break;
            }
        }

        return valid;
    }

    void buildRoofs(ClipperLibXYZ::PolyTree* polyTree, std::vector<std::vector<trimesh::vec3>*>& patches, double roofHeight, double thickness)
    {
        std::vector<PolyPair*> pairs;
        seperate1423(polyTree, pairs);

        for (PolyPair* pair : pairs)
        {
            Polygon_with_holes input;
            build_polygon_with_holes(&input, pair);
            if (!test_simple_polygon(input))
                continue;

            Straight_skeleton_ptr ss = CGAL::create_interior_straight_skeleton_2(input);
            if (ss)
            {

            }
            else
            {
                std::cerr << "ERROR creating interior straight skeleton" << std::endl;
            }
        }

        for (PolyPair* pair : pairs)
        {
            delete pair;
        }
        pairs.clear();
    }

    ClipperLibXYZ::IntPoint cgal_to_point(const Point& point)
    {
        ClipperLibXYZ::IntPoint p;
        p.X = (ClipperLibXYZ::cInt)(1000.0 * point.x());
        p.Y = (ClipperLibXYZ::cInt)(1000.0 * point.y());

        return p;
    }

    void traitSkeletonPoints(ClipperLibXYZ::PolyTree* roofPoint, Straight_skeleton_ptr skeleton)
    {
        for (Vertex_const_iterator vit = skeleton->vertices_begin();
            vit != skeleton->vertices_end(); ++vit)
        {
            Vertex_const_handle h = vit;
            if (h->is_contour() || h->is_skeleton())
                roofPoint->Contour.push_back(cgal_to_point(h->point()));
        }
    }

    void traitSkeletonLine(ClipperLibXYZ::PolyTree* roofLine, Straight_skeleton_ptr skeleton)
    {
        for (Halfedge_const_iterator hit = skeleton->halfedges_begin();
            hit != skeleton->halfedges_end(); ++hit)
        {
            Halfedge_const_handle h = hit;

            roofLine->Contour.push_back(cgal_to_point(h->vertex()->point()));
            roofLine->Contour.push_back(cgal_to_point(h->opposite()->vertex()->point()));
        }
    }

    void traitSkeletonFace(ClipperLibXYZ::Paths* roofFace, Straight_skeleton_ptr skeleton, bool clockwise = false)
    {
        size_t faceSize = skeleton->size_of_faces();
        size_t roofFase = roofFace->size();
        if (faceSize > 0)
            roofFace->resize(roofFase + faceSize);

        int index = roofFase;
        for (Face_const_iterator fit = skeleton->faces_begin();
            fit != skeleton->faces_end(); ++fit)
        {
            ClipperLibXYZ::Path& path = roofFace->at(index);
            //if (index < roofFase + faceSize)
            {
                Halfedge_const_handle he = fit->halfedge();
                Halfedge_const_handle h = he;
                do
                {
                    ClipperLibXYZ::IntPoint p = cgal_to_point(h->vertex()->point());
                    if (h->vertex()->is_skeleton())
                    {
                        p.Z = 500;
                        if (clockwise)
                        {
                            p.Z = 300;
                        }
                    }
                    path.push_back(p);
                    h = h->next();
                } while (h != he);

            }

            ++index;
        }
    }

    void roofLine(ClipperLibXYZ::PolyTree* polyTree,
        ClipperLibXYZ::PolyTree* roof, ClipperLibXYZ::PolyTree* roofPoint, ClipperLibXYZ::Paths* roofFace, bool onePoly)
    {
        std::vector<PolyPair*> pairs;
        if (onePoly)
            seperate1234(polyTree, pairs);
        else
            seperate1423(polyTree, pairs);  

        for (PolyPair* pair : pairs)
        {
            Polygon_with_holes input;
            build_polygon_with_holes(&input, pair);
            if (!test_simple_polygon(input))
                continue;

            ClipperLibXYZ::PolyTree opposite;
            Straight_skeleton_ptr aSkeleton = CGAL::create_interior_straight_skeleton_2(input);
            if (aSkeleton)
            {
                if (roofPoint)
                {
                    traitSkeletonPoints(roofPoint, aSkeleton);
                }

                if (roof)
                {
                    traitSkeletonLine(roof, aSkeleton);
                }

                if (roofFace)
                {
                    traitSkeletonFace(roofFace, aSkeleton, pair->clockwise);
                }                       
            }
            else
            {
                std::cerr << "ERROR creating interior straight skeleton" << std::endl;
            }
        }

        for (PolyPair* pair : pairs)
        {
            delete pair;
        }
        pairs.clear();
    }

    void traitSkeletonPoints(ClipperLibXYZ::Path* roofPoint, Straight_skeleton_ptr skeleton)
    {
        for (Vertex_const_iterator vit = skeleton->vertices_begin();
            vit != skeleton->vertices_end(); ++vit)
        {
            Vertex_const_handle h = vit;
            if (h->is_skeleton())
                roofPoint->push_back(cgal_to_point(h->point()));
        }
    }
    bool havePoint(const ClipperLibXYZ::Path* path, const ClipperLibXYZ::IntPoint& point)
    {
        for (size_t i = 0; i < path->size(); i++)
        {
            if (path->at(i) == point)
                return true;
        }
        return false;
    }
    bool havePoints(const ClipperLibXYZ::Path* path, const ClipperLibXYZ::IntPoint& p1, const ClipperLibXYZ::IntPoint& p2)
    {
        if (path->size() < 2)
            return false;
        for (size_t i = 1; i < path->size(); i++, i++)
        {
            if ((path->at(i) == p1 && path->at(i - 1) == p2)
                || (path->at(i - 1) == p1 && path->at(i) == p2))
                return true;
        }
        return false;
    }
    void traitSkeletonLine(ClipperLibXYZ::Path* roofline, ClipperLibXYZ::Path* roofPoint, Straight_skeleton_ptr skeleton)
    {
        for (Vertex_const_iterator vit = skeleton->vertices_begin();
            vit != skeleton->vertices_end(); ++vit)
        {
            for (Halfedge_const_iterator hit = skeleton->halfedges_begin();
                hit != skeleton->halfedges_end(); ++hit)
            {
                Halfedge_const_handle h = hit;
                //if (h->is_inner_bisector() && h->opposite()->is_inner_bisector())
                if (h->vertex()->is_skeleton() && h->opposite()->vertex()->is_skeleton())
                {
                   const ClipperLibXYZ::IntPoint& p = cgal_to_point(h->vertex()->point());
                   const ClipperLibXYZ::IntPoint& po = cgal_to_point(h->opposite()->vertex()->point());
                    if (havePoint(roofPoint, p) && havePoint(roofPoint, po))
                        if (!havePoints(roofline, p, po))
                        {
                            roofline->push_back(cgal_to_point(h->vertex()->point()));
                            roofline->push_back(cgal_to_point(h->opposite()->vertex()->point()));
                        }
                }
            }
        }
    }

    void skeletonPoints(ClipperLibXYZ::PolyTree* polyTree, ClipperLibXYZ::Path* roofLine)
    {
        ClipperLibXYZ::PolyTree roof;
        std::vector<PolyPair*> pairs;
        seperate1234(polyTree, pairs);
        for (PolyPair* pair : pairs)
        {
            Polygon_with_holes input;
            build_polygon_with_holes(&input, pair);
            if (!test_simple_polygon(input))
                continue;
            ClipperLibXYZ::PolyTree opposite;
            Straight_skeleton_ptr aSkeleton = CGAL::create_interior_straight_skeleton_2(input);
            if (aSkeleton)
            {
                ClipperLibXYZ::Path roofPoint;
                traitSkeletonPoints(&roofPoint, aSkeleton);
                if (roofLine)
                    traitSkeletonLine(roofLine, &roofPoint, aSkeleton);
            }
            else
            {
                std::cerr << "ERROR creating interior straight skeleton" << std::endl;
            }
        }
        for (PolyPair* pair : pairs)
        {
            delete pair;
        }
        pairs.clear();
    }
}


