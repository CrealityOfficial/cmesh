#ifndef FMESH_ROOF_1605318972342_H
#define FMESH_ROOF_1605318972342_H
#include "cmesh/interface.h"
#include "trimesh2/Vec.h"
#include <clipper/clipper.hpp>

//seperate
namespace cmesh
{
	struct PolyPair
	{
		bool clockwise;
		ClipperLib::PolyNode* outer;
		std::vector<ClipperLib::PolyNode*> inner;
	};
	CMESH_API void seperate1423(ClipperLib::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs);
	CMESH_API void seperate1234(ClipperLib::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs);

	CMESH_API void buildRoofs(ClipperLib::PolyTree* polyTree, std::vector<std::vector<trimesh::vec3>*>& patches, double roofHeight, double thickness);

	CMESH_API void roofLine(ClipperLib::PolyTree* polyTree,
		ClipperLib::PolyTree* roof, ClipperLib::PolyTree* roofPoint, ClipperLib::Paths* roofFace, bool onePoly = false);

	CMESH_API void skeletonPoints(ClipperLib::PolyTree* polyTree, ClipperLib::Path* roofPoint);
}

#endif // FMESH_ROOF_1605318972342_H