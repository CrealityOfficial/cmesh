#include "cconversion.h"
#include "ccglobal/tracer.h"

#include "mmesh/trimesh/trimeshutil.h"
#include "tmeshutil.h"
#include "cholefill.h"

namespace cmesh
{
	void removeNorVector2(trimesh::TriMesh* mesh)
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
	void splitTmesh2Cmesh2(trimesh::TriMesh* mesh, std::vector<CMesh>& outMeshes, ccglobal::Tracer* tracer)
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

			if (tracer)
			{
				if (tracer->interrupt())
					return;
			}

			CMesh newMesh1;
			std::vector<CMesh> _outMeshes;
			removeNorVector2(outTMeshes[i]);
			_convertT2C(*outTMeshes[i], newMesh1);

			outMeshes.push_back(newMesh1);
		}

		for (size_t i = 0; i < outTMeshes.size(); i++)
		{
			delete outTMeshes[i];
			outTMeshes[i] = nullptr;
		}
		outTMeshes.clear();
	}

	void dealPerCMesh(CMesh& cmesh, std::vector<CMesh>& newCMesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
	{
		if (!CGAL::is_closed(cmesh))
		{
			try {

				_holeFilling(cmesh, newCMesh, refine_and_fair_hole, tracer);

			}
			catch (std::exception& e)
			{
				std::cerr << e.what();
			}
		}
	}

	void checkoutSelfIntersect2(std::vector<CMesh>& outMeshes, std::vector<bool>& outMeshesSelfIntersect)
	{
		for (int i = 0; i < outMeshes.size(); i++)
		{
			outMeshesSelfIntersect[i] = CGAL::Polygon_mesh_processing::does_self_intersect(outMeshes[i]);
		}
	}

	void selfIntersections2(CMesh& cmesh)
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

	bool checkFill2(std::vector<CMesh>& outMeshes, CMesh& newMesh, CMesh& cmesh, CMesh& cmeshFill, int index, ccglobal::Tracer* tracer)
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
				//bool newMeshSelfIntersect = CGAL::Polygon_mesh_processing::does_self_intersect(cmesh);
				//bool newMeshSelfIntersectFill = CGAL::Polygon_mesh_processing::does_self_intersect(cmeshFill);
				//if (!newMeshSelfIntersect && newMeshSelfIntersectFill)
				//	return false;
				continue;
			}

			std::vector<std::vector<Point>> out;
			CGAL::Polygon_mesh_processing::surface_intersection(newMesh, outMeshes[i], std::back_inserter(out));

			if (out.size() > 0)
			{
				return true;
			}
		}
		return false;
	}


	void dealPerCMeshs(std::vector<CMesh>& inMeshs, std::vector<CMesh>& outMeshs, CMesh& cmesh,int index, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
	{

		if (tracer)
		{
			tracer->progress(0.2f);
		}

		if (tracer)
		{
			if (tracer->interrupt())
				return ;
		}

		std::vector<CMesh> newCMesh;
		CMesh& cmeshFill = cmesh;
		dealPerCMesh(cmeshFill, newCMesh, refine_and_fair_hole, tracer);

		std::vector<bool> outMeshesSelfIntersect(inMeshs.size(), false);
		std::vector<CMesh> noIntersectinMeshs = inMeshs;
		for (size_t i = 0; i < noIntersectinMeshs.size(); i++)
		{
			selfIntersections2(noIntersectinMeshs[i]);
		}


		if (tracer)
		{
			tracer->progress(0.5f);
		}

		if (tracer)
		{
			if (tracer->interrupt())
				return;
		}

		std::vector<std::vector<Point>> out;
		bool bintersect = false;
		for (size_t i = 0; i < newCMesh.size(); i++)
		{
			bintersect=checkFill2(noIntersectinMeshs, newCMesh[i], cmesh, cmeshFill,index, tracer);

			if (!bintersect)
			{
				outMeshs.push_back(newCMesh[i]);
			}
		}

		if (tracer)
		{
			tracer->progress(1.0f);
		}

		if (tracer)
		{
			if (tracer->interrupt())
				return;
		}
	}

	trimesh::TriMesh* repairMenuNew(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
	{
		if (tracer)
		{
			tracer->progress(0.2f);
		}

		if (tracer)
		{
			if (tracer->interrupt())
				return nullptr;
		}

		std::vector<CMesh> outMeshes;
		splitTmesh2Cmesh2(mesh, outMeshes, tracer);
		if (outMeshes.size() == 0)
			return nullptr;

		if (tracer)
		{
			tracer->progress(0.4f);
		}

		if (tracer)
		{
			if (tracer->interrupt())
				return nullptr;
		}

		std::vector<CMesh> outCMeshes;
		for (int i=0; i< outMeshes.size(); i++)
		{
			outCMeshes.push_back(outMeshes[i]);
			dealPerCMeshs(outMeshes, outCMeshes, outMeshes[i],i, refine_and_fair_hole, tracer);
		}

		if (tracer)
		{
			tracer->progress(0.6f);
		}

		if (tracer)
		{
			if (tracer->interrupt())
				return nullptr;
		}

		std::vector<trimesh::TriMesh*> meshes;
		for (size_t i = 0; i < outCMeshes.size(); i++)
		{
			trimesh::TriMesh* mergedMeshOutward = new trimesh::TriMesh();
			_convertC2T(outCMeshes[i], *mergedMeshOutward);
			meshes.push_back(mergedMeshOutward);
		}


		if (tracer)
		{
			tracer->progress(0.8f);
		}

		if (tracer)
		{
			if (tracer->interrupt())
				return nullptr;
		}

		trimesh::TriMesh* newMesh = new trimesh::TriMesh();
		mmesh::mergeTriMesh(newMesh, meshes);

		for (size_t i = 0; i < meshes.size(); i++)
		{
			delete meshes[i];
		}
		meshes.clear();

		CMesh cmeshA;
		_convertT2C(*newMesh, cmeshA);
		if (newMesh)
		{
			delete newMesh;
			newMesh = nullptr;
		}
		if (CGAL::is_closed(cmeshA))
		{
			CGAL::Polygon_mesh_processing::orient(cmeshA);
		}

		try {
			if (!CGAL::Polygon_mesh_processing::is_outward_oriented(cmeshA))
			{
				CGAL::Polygon_mesh_processing::reverse_face_orientations(cmeshA);
			}
		}
		catch (std::exception& e)
		{
			std::cerr << e.what();
		}

		newMesh = new trimesh::TriMesh();
		_convertC2T(cmeshA, *newMesh);


		if (tracer)
		{
			tracer->progress(1.0f);
		}

		if (tracer)
		{
			if (tracer->interrupt())
				return nullptr;
		}

		return newMesh;
	}
}