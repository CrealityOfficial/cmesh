#ifndef FMESH_ROOF_1605318972342_H
#define FMESH_ROOF_1605318972342_H
#include "cmesh/interface.h"
#include "trimesh2/Vec.h"
#include "clipperxyz/clipper.hpp"

//seperate
namespace cmesh
{
	struct PolyPair
	{
		bool clockwise;
		ClipperLibXYZ::PolyNode* outer;
		std::vector<ClipperLibXYZ::PolyNode*> inner;
	};
	CMESH_API void seperate1423(ClipperLibXYZ::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs);
	CMESH_API void seperate1234(ClipperLibXYZ::PolyTree* polyTree, std::vector<PolyPair*>& polyPairs);

	CMESH_API void buildRoofs(ClipperLibXYZ::PolyTree* polyTree, std::vector<std::vector<trimesh::vec3>*>& patches, double roofHeight, double thickness);

	CMESH_API void roofLine(ClipperLibXYZ::PolyTree* polyTree,
		ClipperLibXYZ::PolyTree* roof, ClipperLibXYZ::PolyTree* roofPoint, ClipperLibXYZ::Paths* roofFace, bool onePoly = false);

	CMESH_API void skeletonPoints(ClipperLibXYZ::PolyTree* polyTree, ClipperLibXYZ::Path* roofPoint);
}

#endif // FMESH_ROOF_1605318972342_H