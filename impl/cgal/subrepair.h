#ifndef CMESH_SUBREPAIR_1648026566432_H
#define CMESH_SUBREPAIR_1648026566432_H
#include "ccglobal/tracer.h"
#include "trimesh2/TriMesh.h"
#include "../ctype.h"
#include "cmesh/cgal/subrepairtype.h"

namespace cmesh
{
    void CGALisotropicRemeshing(CMesh& cmesh, double target_edge_length, ccglobal::Tracer* trace = nullptr);

    //根据角度三角化
    void CGALsmoothing(CMesh& cmesh, double angle, ccglobal::Tracer* trace = nullptr);

    //细化两个模型的相交区域
    void CGALcorefinement(CMesh& cmesh1, CMesh& cmesh2, ccglobal::Tracer* trace = nullptr);

    //三角化:局部优化
    void CGALtriangulate(CMesh& cmesh1, ccglobal::Tracer* trace = nullptr);

    //三角化:修复自交问题
    void CGALtangential(CMesh& cmesh1, ccglobal::Tracer* trace = nullptr);

    //boolean
    void CGALboolean(CMesh& cmesh1, CMesh& cmesh2, CMesh& cmeshOut, CGALBooleanType type, ccglobal::Tracer* trace = nullptr);

    //hole fill
    void CGALholeFill(CMesh& cmesh, CGALHoleFillType type, ccglobal::Tracer* trace = nullptr);

    //intersect
    bool CGALselfIntersections(CMesh& cmesh, std::vector<int>& faceIndex, bool dealFace = true, ccglobal::Tracer* trace = nullptr);

    //orientation
    void CGALorientation(CMesh& cmesh, bool reversible = false, ccglobal::Tracer* trace = nullptr);

    //combinatorial  stitch
    void CGALstitch(CMesh& cmesh, ccglobal::Tracer* trace = nullptr);
    //combinatorial  stitch
    void CGALmanifoldness(CMesh& cmesh, ccglobal::Tracer* trace = nullptr);

    //connected
    void CGALconnectedComponents(CMesh& cmesh, ccglobal::Tracer* trace = nullptr);

    //detect_features
    void CGALdetectFeatures(CMesh& cmesh, ccglobal::Tracer* trace = nullptr);

}
#endif // CMESH_SUBREPAIR_1648026566432_H