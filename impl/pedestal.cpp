#include "cmesh/mesh/pedestal.h"
#include "cholefill.h"
#include "tmeshutil.h"
#include "cconversion.h"
#include "repairNew.h"

#include "mmesh/trimesh/trimeshutil.h"
#include "mmesh/util/dumplicate.h"

namespace cmesh
{
	trimesh::TriMesh* pedestalMenu(trimesh::TriMesh* mesh, pedestalParam apram, ccglobal::Tracer* tracer)
	{
		if (tracer) tracer->progress(0.2f);
		if (tracer)
		{
			if (tracer->interrupt())
				return nullptr;
		}

		float fZ = 0.0;
		if (apram.adirection== Direction::down)
		{
			mesh->need_bbox();
			float fheight = apram.height - mesh->bbox.min.z;
			for (trimesh::point& apoint : mesh->vertices)
			{
				apoint.z += fheight;
			}
		}
		else//uper
		{
			fZ = apram.height + mesh->bbox.max.z;
		}

		std::vector<CMesh> inputcmeshs;
		splitTmesh2Cmesh2(mesh, inputcmeshs, tracer);
		if (inputcmeshs.size() == 0)
			return nullptr;

		if (tracer) tracer->progress(0.4f);
		if (tracer)
		{
			if (tracer->interrupt())
				return nullptr;
		}

		for (int i = 0; i < inputcmeshs.size(); i++)
		{
			if (!CGAL::is_closed(inputcmeshs[i]))
			{
				try 
				{
					pedestalFilling(inputcmeshs[i],fZ,apram.isSmooth,tracer);
				}
				catch (std::exception& e)
				{
					std::cerr << e.what();
				}
			}
		}

		std::vector<trimesh::TriMesh*> output;//cmesh转换为trimesh
		for (int i = 0; i < inputcmeshs.size(); i++)
		{
			trimesh::TriMesh* mergedMeshOutward = new trimesh::TriMesh();
			_convertC2T(inputcmeshs[i], *mergedMeshOutward);
			output.push_back(mergedMeshOutward);
		}

		if (tracer)	tracer->progress(0.8f);
		if (tracer)
		{
			if (tracer->interrupt())
				return nullptr;
		}

		trimesh::TriMesh* output2 = new trimesh::TriMesh();//合并多个output 为一个trimesh
		mmesh::mergeTriMesh(output2, output);
		return output2;
	}
}