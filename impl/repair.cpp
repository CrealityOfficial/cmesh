#include "cmesh/mesh/repair.h"
#include "cholefill.h"
#include "tmeshutil.h"
#include "cconversion.h"

#include "mmesh/trimesh/trimeshutil.h"
#include "mmesh/util/dumplicate.h"

namespace cmesh
{
	void getErrorInfoRich(RichMesh& mesh, ErrorInfo& info)
	{
	}

	void getErrorInfo(trimesh::TriMesh* mesh, ErrorInfo& info)
	{
		info.edgeNum = 0;
		info.normalNum = 0;
		info.holeNum = 0;
		info.intersectNum = 0;

		CMesh cmesh;
		_convertT2C(*mesh, cmesh);

		int normalNum = 0;
		for (halfedge_descriptor h : halfedges(cmesh))
		{
			if (is_border(h, cmesh))
			{
				info.edgeNum++;
			}
			else if (is_isolated_triangle(h, cmesh))
			{
				normalNum++;
			}
		}
		if (normalNum==0 && CGAL::is_closed(cmesh))
		{
			if (!PMP::is_outward_oriented(cmesh))
				info.normalNum = cmesh.faces().size();
		}
		else
			info.normalNum = normalNum / 3; 

		std::vector<halfedge_descriptor> border_cycles;
		PMP::extract_boundary_cycles(cmesh, std::back_inserter(border_cycles));
		for (halfedge_descriptor h : border_cycles)
		{
			info.holeNum++;
		}

#if 0  //检测影响加载时间
		bool intersecting = PMP::does_self_intersect<CGAL::Parallel_if_available_tag>(cmesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, cmesh)));
		std::vector<std::pair<face_descriptor, face_descriptor> > intersected_tris;
		if (intersecting)
		{
			PMP::self_intersections<CGAL::Parallel_if_available_tag>(faces(cmesh), cmesh, std::back_inserter(intersected_tris));
		}
		info.intersectNum = intersected_tris.size();
#endif
	}

	void selfIntersections(CMesh& cmesh)
	{
		bool intersecting = PMP::does_self_intersect<CGAL::Parallel_if_available_tag>(cmesh, CGAL::parameters::vertex_point_map(get(CGAL::vertex_point, cmesh)));
		if (!intersecting)
		{
			return;
		}

		//std::string npath1 = "selfIntersections1.off";
		//CGAL::IO::write_polygon_mesh(npath1, cmesh, CGAL::parameters::stream_precision(17));

		std::vector<std::pair<face_descriptor, face_descriptor> > intersected_tris;
		PMP::self_intersections<CGAL::Parallel_if_available_tag>(faces(cmesh), cmesh, std::back_inserter(intersected_tris));
		std::vector<int>delFace;
		for (int i = 0; i < intersected_tris.size(); ++i)
		{
			delFace.push_back(intersected_tris[i].first);
			delFace.push_back(intersected_tris[i].second);
		}
		std::sort(delFace.begin(), delFace.end());
		delFace.erase(std::unique(delFace.begin(), delFace.end()), delFace.end());

		CMesh cmeshnew;
		for (Point v : cmesh.points())
		{
			cmeshnew.add_vertex(v);
		}
		for (CMesh::Face_index face_index : cmesh.faces())
		{
			CGAL::Vertex_around_face_circulator<CMesh> vcirc(cmesh.halfedge(face_index), cmesh), done(vcirc);

			if (delFace.end() == std::find(delFace.begin(), delFace.end(), face_index))
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

		//npath1 = "selfIntersections2.off";
		//CGAL::IO::write_polygon_mesh(npath1, cmesh, CGAL::parameters::stream_precision(17));
	}

	void removeNorVector(trimesh::TriMesh* mesh)
	{
		if (!mesh)
		{
			return;
		}

		int nf = mesh->faces.size();

		std::vector<char> isIsolated(mesh->vertices.size(), 0);
#if defined(_OPENMP)
#pragma omp parallel for
#endif
		for (int i = 0; i < nf; i++) {
			const int& f1 = mesh->faces[i].x;
			const int& f2 = mesh->faces[i].y;
			const int& f3 = mesh->faces[i].z;
			if (f1 != f2 && f1 != f3 && f2 != f3)
			{
				isIsolated[mesh->faces[i].x] = 1;
				isIsolated[mesh->faces[i].y] = 1;
				isIsolated[mesh->faces[i].z] = 1;
			}
		}

		std::vector<trimesh::point> validVertices;
		std::vector<int> V2newV(mesh->vertices.size(), 0);
		for (int i = 0; i < mesh->vertices.size(); i++)
		{
			if (isIsolated[i] == 0)
			{
				V2newV[i] = -1;
			}
			else
			{
				validVertices.push_back(mesh->vertices[i]);
				V2newV[i] = validVertices.size() - 1;
			}
		}

		//mesh->need_neighbors();
		//mesh->neighbors;

		std::vector<trimesh::TriMesh::Face> validFaces;
		for (int i = 0; i < nf; i++) {
			if (V2newV[mesh->faces[i].x] != -1
				&& V2newV[mesh->faces[i].y] != -1
				&& V2newV[mesh->faces[i].z] != -1)
			{
				validFaces.push_back(trimesh::TriMesh::Face(V2newV[mesh->faces[i].x]
					, V2newV[mesh->faces[i].y]
					, V2newV[mesh->faces[i].z]));
			}
		}
		mesh->faces.swap(validFaces);
		mesh->vertices.swap(validVertices);
	}

	void splitTmesh2Cmesh(trimesh::TriMesh* mesh, std::vector<CMesh>& outMeshes, ccglobal::Tracer* tracer)
	{
		//split step1:
		std::vector<trimesh::TriMesh*> outTMeshes;
		spiltModel(mesh, outTMeshes, tracer);

		//split step2:
		float countNums = outTMeshes.size() == 0 ? 1 : 1.0 / outTMeshes.size();
		int count = 1;
		for (size_t i = 0; i < outTMeshes.size(); i++)
		{
			if (tracer)
			{
				tracer->progress(countNums * count++);
			}
			CMesh newMesh1;
			std::vector<CMesh> _outMeshes;
			removeNorVector(outTMeshes[i]);
			_convertT2C(*outTMeshes[i], newMesh1);

			//std::vector<std::vector<vertex_descriptor> > duplicated_vertices;
			//std::size_t new_vertices_nb = PMP::duplicate_non_manifold_vertices(newMesh1,
			//	NP::output_iterator(
			//		std::back_inserter(duplicated_vertices)));

			//CGAL::Polygon_mesh_processing::split_connected_components(newMesh1, _outMeshes);

			//selfIntersections(newMesh1);

			//outMeshes.insert(outMeshes.end(), _outMeshes.begin(), _outMeshes.end());

			outMeshes.push_back(newMesh1);
		}
	}


	void repairHole(RichMesh& mesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
	{
		CMesh& cmesh = mesh.impl().mesh;
		_holeFilling(cmesh, refine_and_fair_hole, tracer);
	}

	bool checkFill(std::vector<CMesh>& outMeshes, CMesh newMesh, int index, ccglobal::Tracer* tracer)
	{
		float countNums = outMeshes.size() == 0 ? 1 : 1.0 / outMeshes.size();
		int count = 1;
		for (int i = 0; i < outMeshes.size(); i++)
		{
			if (tracer)
			{
				tracer->progress(countNums * count++);
			}

			if (index == i)
			{
				if (CGAL::Polygon_mesh_processing::does_self_intersect(newMesh)
					&& !CGAL::Polygon_mesh_processing::does_self_intersect(outMeshes[1]))
					return false;
				continue;
			}

			std::vector<std::vector<Point>> out;
			if (!CGAL::Polygon_mesh_processing::does_self_intersect(newMesh)
				&& !CGAL::Polygon_mesh_processing::does_self_intersect(outMeshes[1]))
				CGAL::Polygon_mesh_processing::surface_intersection(newMesh, outMeshes[1], std::back_inserter(out));

			if (out.size() > 0)
			{
				return false;
			}
		}
		return true;
	}

	void splitByOrientedAndHoleFill(std::vector<CMesh>& outMeshes
		, std::vector<CMesh>& coutward_oriented
		, std::vector<CMesh>& cinside_oriented
		, bool refine_and_fair_hole
		, ccglobal::Tracer* tracer)
	{
		float countNums = outMeshes.size() == 0 ? 1 : 1.0 / outMeshes.size();
		int count = 1;
		for (int i = 0; i < outMeshes.size(); i++)
		{
			if (tracer)
			{
				tracer->progress(countNums * count++);
			}

			CMesh newMesh = outMeshes[i];

			if (!CGAL::is_closed(newMesh))
			{
				try {

					_holeFilling(newMesh, refine_and_fair_hole, tracer);

				}
				catch (std::exception& e)
				{
					std::cerr << e.what();
				}
			}

			bool result = checkFill(outMeshes, newMesh, i, tracer);

			//CGAL::Polygon_mesh_processing::orient(newMesh);
			try {
				if (CGAL::is_closed(newMesh))
				{
					CGAL::Polygon_mesh_processing::orient(newMesh);
				}

				if (CGAL::Polygon_mesh_processing::is_outward_oriented(newMesh))
				{
					//CGAL::Polygon_mesh_processing::orient(newMesh);
					if (!result)
					{
						coutward_oriented.push_back(outMeshes[i]);
					}
					else
						coutward_oriented.push_back(newMesh);
				}
				else
				{
					if (!result)
					{
						coutward_oriented.push_back(outMeshes[i]);
					}
					else
					{
						CGAL::Polygon_mesh_processing::reverse_face_orientations(newMesh);
						//CGAL::Polygon_mesh_processing::orient(newMesh);
						cinside_oriented.push_back(newMesh);
					}
				}
			}
			catch (std::exception& e)
			{
				std::cerr << e.what();
			}

		}
	}

	void orientedDetect(CMesh& cmesh, ccglobal::Tracer* tracer)
	{
		try {
			if (CGAL::is_closed(cmesh)&& !CGAL::Polygon_mesh_processing::is_outward_oriented(cmesh))
			{
				CGAL::Polygon_mesh_processing::reverse_face_orientations(cmesh);
			}
		}
		catch (std::exception& e)
		{
			std::cerr << e.what();
		}
	}

	void HoleFill(CMesh& cmesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
	{
		if (!CGAL::is_closed(cmesh))
		{
			try {
				_holeFilling(cmesh, refine_and_fair_hole, tracer);
			}
			catch (std::exception& e)
			{
				std::cerr << e.what();
			}
		}
	}

	void unionByOriented(std::vector<trimesh::TriMesh*>& meshes, std::vector<CMesh>& coutward_oriented, bool isOutwardoriented, ccglobal::Tracer* tracer)
	{
		//CMesh coutward_union;
		//unionByBoolean(coutward_oriented, coutward_union, tracer);

		std::vector<CMesh> coutward_unions;
		//unionByBooleanThread(coutward_oriented, coutward_unions, tracer);
		coutward_unions = coutward_oriented;

		float countNums = coutward_unions.size() == 0 ? 1 : 1.0 / coutward_unions.size();
		int count = 1;

		for (size_t i = 0; i < coutward_unions.size(); i++)
		{
			if (tracer)
			{
				tracer->progress(countNums * count++);
			}
			CMesh& coutward_union = coutward_unions[i];
			orientedDetect(coutward_union, tracer);

			if (!isOutwardoriented)
			{
				CGAL::Polygon_mesh_processing::reverse_face_orientations(coutward_union);
			}
			trimesh::TriMesh* mergedMeshOutward = new trimesh::TriMesh();
			_convertC2T(coutward_union, *mergedMeshOutward);
			meshes.push_back(mergedMeshOutward);
		}
	}

	void repairMenuRich(RichMesh& meshInput, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
	{
	}

	trimesh::TriMesh* repairMenu(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
	{
		//if (CGAL::is_closed(meshInput.impl().mesh))
		//{
		//	CGAL::Polygon_mesh_processing::orient(meshInput.impl().mesh);
		//}

		//orientedDetect(meshInput.impl().mesh, tracer);

		//trimesh::TriMesh* mesh = meshInput.generateTrimesh();

		if (tracer)
		{
			tracer->progress(0.2f);
		}

		std::vector<CMesh> outMeshes;
		splitTmesh2Cmesh(mesh, outMeshes, tracer);
		if (outMeshes.size() == 0)
			return nullptr;

		if (tracer)
		{
			tracer->progress(0.4f);
		}

		trimesh::TriMesh* newMesh = new trimesh::TriMesh();

		CMesh cmesh;//output
		if (outMeshes.size() > 1)
		{
			std::vector<CMesh> coutward_oriented;
			std::vector<CMesh> cinside_oriented;
			splitByOrientedAndHoleFill(outMeshes, coutward_oriented, cinside_oriented, refine_and_fair_hole,tracer);

			if (tracer)
			{
				tracer->progress(0.6f);
			}

			std::vector<trimesh::TriMesh*> meshes;
			if (coutward_oriented.size() > 0) {
				unionByOriented(meshes, coutward_oriented, true, tracer);
			}
			if (cinside_oriented.size() > 0) {
				if (coutward_oriented.size() == 0)
					unionByOriented(meshes, cinside_oriented, true, tracer);
				else
					unionByOriented(meshes, cinside_oriented, false, tracer);
			}

			if (tracer)
			{
				tracer->progress(0.8f);
			}

			//newMesh->clear();
			mmesh::mergeTriMesh(newMesh, meshes);

			for (size_t i = 0; i < meshes.size(); i++)
			{
				delete meshes[i];
			}
			meshes.clear();
		}
		else
		{
			CMesh& cmesh = outMeshes.front();
			//_convertT2C(outMeshes.front(),cmesh);
			HoleFill(cmesh, refine_and_fair_hole,tracer);
			orientedDetect(cmesh, tracer);
			_convertC2T(cmesh, *newMesh);
		}

		if (tracer)
		{
			tracer->progress(1.0f);
		}

		newMesh->need_normals();
		return newMesh;
	}
}