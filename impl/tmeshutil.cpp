#include "tmeshutil.h"

#include "mmesh/trimesh/meshtopo.h"
#include "mmesh/trimesh/trimeshutil.h"

#include "ccglobal/tracer.h"

void spiltModel(trimesh::TriMesh* mesh, std::vector<trimesh::TriMesh*>& validMeshes, ccglobal::Tracer* tracer)
{
	if (!mesh || mesh->faces.size() <= 0)
	{
		return;
	}

	if (tracer)
	{
		tracer->progress(0.1);
	}

	mmesh::MeshTopo topo;
	topo.build(mesh);
	if (tracer)
	{
		tracer->progress(0.3);

		if (tracer->interrupt())
		{
			return;
		}
	}

	int faceNum = (int)mesh->faces.size();
	std::vector<bool> visitFlags(faceNum, false);

	std::vector<int> visitStack;
	std::vector<int> nextStack;

	std::vector<std::vector<int>> parts;
	parts.reserve(100);
	for (int faceID = 0; faceID < faceNum; ++faceID)
	{
		if (visitFlags.at(faceID) == false)
		{
			visitFlags.at(faceID) = true;
			visitStack.push_back(faceID);

			std::vector<int> facesChunk;
			facesChunk.push_back(faceID);
			while (!visitStack.empty())
			{
				int seedSize = (int)visitStack.size();
				for (int seedIndex = 0; seedIndex < seedSize; ++seedIndex)
				{
					int cFaceID = visitStack.at(seedIndex);
					trimesh::ivec3& oppoHalfs = topo.m_oppositeHalfEdges.at(cFaceID);
					for (int halfID = 0; halfID < 3; ++halfID)
					{
						int oppoHalf = oppoHalfs.at(halfID);
						if (oppoHalf >= 0)
						{
							int oppoFaceID = topo.faceid(oppoHalf);
							if (visitFlags.at(oppoFaceID) == false)
							{
								nextStack.push_back(oppoFaceID);
								facesChunk.push_back(oppoFaceID);
								visitFlags.at(oppoFaceID) = true;
							}
						}
					}
				}

				visitStack.swap(nextStack);
				nextStack.clear();
			}

			parts.push_back(std::vector<int>());
			parts.back().swap(facesChunk);
		}
		else
		{
			visitFlags.at(faceID) = true;
		}

		if (tracer)
		{
			if ((faceID + 1) % 100 == 0)
			{
				if (tracer->interrupt())
				{
					return;
				}
			}
		}
	}

	std::vector<trimesh::TriMesh*> meshes;
	size_t partSize = parts.size();
	for (size_t i = 0; i < partSize; ++i)
	{
		if (parts.at(i).size() > 10)
		{
			trimesh::TriMesh* outMesh = mmesh::partMesh(parts.at(i), mesh);
			if (outMesh) meshes.push_back(outMesh);
		}
	}

	//merge small ones
	int tSize = (int)meshes.size();
	int maxCount = 0;
	for (int i = 0; i < tSize; ++i)
	{
		if (maxCount < (int)meshes.at(i)->vertices.size())
			maxCount = (int)meshes.at(i)->vertices.size();
	}

	int smallCount = (int)((float)maxCount * 0.05f);
	std::vector<trimesh::TriMesh*> allInOne;
	//std::vector<trimesh::TriMesh*> validMeshes;
	for (int i = 0; i < tSize; ++i)
	{
		//if ((int)meshes.at(i)->vertices.size() < smallCount)
		//	allInOne.push_back(meshes.at(i));
		//else
			validMeshes.push_back(meshes.at(i));
	}

	if (allInOne.size() > 0)
	{
		trimesh::TriMesh* newMesh = new trimesh::TriMesh();
		mmesh::mergeTriMesh(newMesh, allInOne);
		validMeshes.push_back(newMesh);

		for (trimesh::TriMesh* m : allInOne)
			delete m;
		allInOne.clear();
	}
	//outMeshes.push_back(validMeshes);
	if (tracer)
	{
		tracer->progress(1);
	}
}