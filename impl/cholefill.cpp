#include "cholefill.h"
#include "mmesh/trimesh/polygonstack.h"
#include "mmesh/trimesh/savepolygonstack.h"
#include "repairNew.h"
#include "clipper/clipper.hpp"
#include <algorithm>

namespace cmesh
{
    bool _holeFilling(CMesh& cmesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
    {
        if (tracer)
        {
            tracer->progress(0.4f);
        }

        // Both of these must be positive in order to be considered
        double max_hole_diam = 0.1;
        int max_num_hole_edges = 1;

        std::vector<halfedge_descriptor> border_cycles;
        // collect one halfedge per boundary cycle
        PMP::extract_boundary_cycles(cmesh, std::back_inserter(border_cycles));

        float countNums = border_cycles.size() == 0 ? 1 : 1.0 / border_cycles.size();
        int count = 1;

        for (halfedge_descriptor h : border_cycles)
        {
            if (tracer)
            {
                tracer->progress(countNums * count++);
            }

            std::vector<face_descriptor>  patch_facets;
            std::vector<vertex_descriptor> patch_vertices;
            if (refine_and_fair_hole)
            {

                bool success = std::get<0>(PMP::triangulate_refine_and_fair_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices)));
            }
            else
            {
                PMP::triangulate_and_refine_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices));
            }       
        }

        if (tracer)
        {
            tracer->progress(1.0f);
        }
        return true;
    }

    bool _holeFilling(CMesh& cmesh, std::vector<CMesh>& cmeshs, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
    {
        if (tracer)
        {
            tracer->progress(0.4f);
        }

        // Both of these must be positive in order to be considered
        double max_hole_diam = 0.1;
        int max_num_hole_edges = 1;

        std::vector<halfedge_descriptor> border_cycles;
        // collect one halfedge per boundary cycle
        PMP::extract_boundary_cycles(cmesh, std::back_inserter(border_cycles));

        float countNums = border_cycles.size() == 0 ? 1 : 1.0 / border_cycles.size();
        int count = 1;

        for (halfedge_descriptor h : border_cycles)
        {
            if (tracer)
            {
                tracer->progress(countNums * count++);
            }

            std::vector<face_descriptor>  patch_facets;
            std::vector<vertex_descriptor> patch_vertices;
            if (refine_and_fair_hole)
            {

                bool success = std::get<0>(PMP::triangulate_refine_and_fair_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices)));
            }
            else
            {
                PMP::triangulate_and_refine_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices));
            }

            if (patch_facets.size())
            {
                CMesh newCmesh;
                std::map<int, int> v2v;
                auto insertCMesh = [&cmesh](std::map<int, int>& v2v, CMesh& newCmesh, int index, int& faceIndex) {
                    auto iter = v2v.find(index);
                    if (iter == v2v.end())
                    {
                        vertex_descriptor indexd(index);
                        int n0 = indexd.idx();
                        faceIndex = newCmesh.add_vertex(cmesh.point(indexd));
                        v2v.insert(std::pair<int, int>(index, faceIndex));
                    }
                    else
                    {
                        faceIndex = v2v.at(index);
                    }
                };

                for (face_descriptor Ff : patch_facets)
                {
                    CGAL::Vertex_around_face_circulator<CMesh> vcirc(cmesh.halfedge(Ff), cmesh), done(vcirc);
                    int f1;
                    insertCMesh(v2v, newCmesh, (*vcirc).idx(), f1);
                    ++vcirc;
                    int f2;
                    insertCMesh(v2v, newCmesh, (*vcirc).idx(), f2);
                    ++vcirc;
                    int f3;
                    insertCMesh(v2v, newCmesh, (*vcirc).idx(), f3);

                    newCmesh.add_face(CGAL::SM_Vertex_index(f1), CGAL::SM_Vertex_index(f2), CGAL::SM_Vertex_index(f3));
                }

                cmeshs.push_back(newCmesh);
                //int index = h.idx();
                //std::string str = "f:/fill_";
                //str += std::to_string(index);
                //str += ".off";
                //CGAL::IO::write_polygon_mesh(str, newCmesh, CGAL::parameters::stream_precision(17));
            }

        }

        if (tracer)
        {
            tracer->progress(1.0f);
        }
        return true;
    }

	bool pedestalFilling(CMesh& cmesh, float fZ, bool isSmooth, ccglobal::Tracer* tracer)
	{
		if (tracer) tracer->progress(0.4f);
		std::vector<halfedge_descriptor> border_cycles;
		PMP::extract_boundary_cycles(cmesh, std::back_inserter(border_cycles));

		std::vector<trimesh::TriMesh::Face> triangles;
		for (halfedge_descriptor ahalfedge : border_cycles)
		{
			std::vector<Point> Vertexes;
			std::vector<vertex_descriptor> indexes;

			CGAL::Halfedge_around_face_circulator<CMesh>  circ1(ahalfedge, cmesh), done(circ1);
			do
			{
				vertex_descriptor  avertex_descriptor = target(*circ1, cmesh);
				Point aPoint_3 = cmesh.point(avertex_descriptor);
				Vertexes.push_back(aPoint_3);
				indexes.push_back(avertex_descriptor);
			} while (++circ1 != done);

			int vNum = cmesh.num_vertices();

			vertex_descriptor prevIndex, prevZ0Index, currentZ0Index, firstZ0Index;
			for (int n = 0; n < indexes.size(); n++)
			{
				if (n == 0)
				{
					currentZ0Index = cmesh.add_vertex(Point(Vertexes.at(n).x(), Vertexes.at(n).y(), fZ));
					firstZ0Index = currentZ0Index;

					prevZ0Index = currentZ0Index;
					prevIndex = indexes[n];
				}
				else
				{
					currentZ0Index = cmesh.add_vertex(Point(Vertexes.at(n).x(), Vertexes.at(n).y(), fZ));

					cmesh.add_face(prevZ0Index, prevIndex, indexes[n]);
					cmesh.add_face(prevZ0Index, indexes[n], currentZ0Index);

					prevZ0Index = currentZ0Index;
					prevIndex = indexes[n];
				}
			}
			cmesh.add_face(prevZ0Index, prevIndex, indexes[0]);
			cmesh.add_face(prevZ0Index, indexes[0], firstZ0Index);
		}


		border_cycles.clear();
		PMP::extract_boundary_cycles(cmesh, std::back_inserter(border_cycles));
        for (halfedge_descriptor h : border_cycles)
        {
			std::vector<trimesh::dvec2> points;//µãÊý¾Ý
			CGAL::Halfedge_around_face_circulator<CMesh>  circ1(h, cmesh), done(circ1);
			do
			{
				vertex_descriptor  avertex_descriptor = target(*circ1, cmesh);
				Point aPoint_3 = cmesh.point(avertex_descriptor);
				points.push_back(trimesh::dvec2(aPoint_3.x(), aPoint_3.y()));
			} while (++circ1 != done);

            ClipperLib::Path apath;
			for (trimesh::dvec2 apoint : points)
			{
				apath.push_back(ClipperLib::IntPoint(apoint.x*100.0, apoint.y * 100.0));
			}
            points.clear();

            ClipperLib::Paths outPaths;
			if (0)
			{
				ClipperLib::Paths apaths;
				apaths.push_back(apath);
				ClipperLib::Clipper alipper;
				ClipperLib::PolyTree retval;
				alipper.AddPath(apath, ClipperLib::ptClip, true);
				alipper.Execute(ClipperLib::ClipType::ctUnion, retval, ClipperLib::pftEvenOdd, ClipperLib::pftEvenOdd);
				ClipperLib::PolyTreeToPaths(retval, outPaths);
			}
			else
			{
				ClipperLib::SimplifyPolygon(apath, outPaths, ClipperLib::pftEvenOdd);
			}

            std::vector<std::vector<int>> polygonses;
			int icount = 0;
			for (ClipperLib::Path& apath : outPaths)
			{
                polygonses.push_back(std::vector<int>());
				for (ClipperLib::IntPoint& apoint : apath)
				{
                    polygonses[polygonses.size() - 1].push_back(icount++);
                    points.push_back(trimesh::dvec2(apoint.X, apoint.Y));
				}
			}

			mmesh::PolygonStack apoly;
			apoly.generatesWithoutTree(polygonses, points, triangles);
            vertex_descriptor firstZ0Index;
			for (int n=0;n<points.size();++n)
			{
                if (n==0)
				{
					firstZ0Index = cmesh.add_vertex(Point(points[n].x/100.0, points[n].y / 100.0, fZ));
                }
                else
                {
                    cmesh.add_vertex(Point(points[n].x / 100.0, points[n].y / 100.0, fZ));
                }
            }

			for (int n = 0; n < triangles.size(); n++)
			{
				cmesh.add_face(vertex_descriptor(triangles[n].at(0)+ firstZ0Index), vertex_descriptor(triangles[n].at(1) + firstZ0Index), vertex_descriptor(triangles[n].at(2) + firstZ0Index));
			}

        }

		if (tracer)
		{
			tracer->progress(1.0f);
		}
		return true;
	}

}