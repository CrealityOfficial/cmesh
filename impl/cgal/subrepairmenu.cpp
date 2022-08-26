#include "cmesh/cgal/subrepairmenu.h"
#include "subrepair.h"
#include "../cconversion.h"
#include "../ctype.h"

//TODO:test error data
//#include "cmesh/mesh/repair.h"

namespace cmesh
{
    trimesh::TriMesh* subRepairMenu(trimesh::TriMesh* mesh1, trimesh::TriMesh* mesh2, RepairMenuParamIn& paramIn, ccglobal::Tracer* tracer)
    {
        //in
        CMesh cmesh1;
        _convertT2CForNoRepair(*mesh1, cmesh1);
        CMesh cmesh2;
        if (mesh2)
        {
            _convertT2CForNoRepair(*mesh2, cmesh1);
        }
        CMesh cmeshOut;

        //type
        switch (paramIn.repairType)
        {
        case CGALRepairType::CGAL_BOOLEAN:
            CGALboolean(cmesh1, cmesh2, cmeshOut, paramIn.booleanType,tracer);
            break;
        case CGALRepairType::CGAL_CONNECTED:

            break;
        case CGALRepairType::CGAL_COREFINE:
            CGALcorefinement(cmesh1, cmesh2, tracer);

            break;
        case CGALRepairType::CGAL_ISOTROPIC:
            CGALisotropicRemeshing(cmesh1, paramIn.target_edge_length, tracer);
            break;
        case CGALRepairType::CGAL_MANIFOLDNESS:
            CGALmanifoldness(cmesh1, tracer);
            break;
        case CGALRepairType::CGAL_ORIENTATION:
            CGALorientation(cmesh1, paramIn.reversible, tracer);
            break;
        case CGALRepairType::CGAL_SELFINTERSECT:
            CGALselfIntersections(cmesh1, paramIn.faceIndex,true,tracer);
            break;
        case CGALRepairType::CGAL_STITCH:
            CGALstitch(cmesh1, tracer);
            break;
        case CGALRepairType::CGAL_HOLE:
            CGALholeFill(cmesh1, paramIn.holeFillType, tracer);
            break;
        case CGALRepairType::CGAL_TRIANGULATE:
            CGALtriangulate(cmesh1, tracer);
            break;
        case CGALRepairType::CGAL_FEATURES:
            CGALdetectFeatures(cmesh1, tracer);
            break;
        case CGALRepairType::CGAL_SMOTHING:
            CGALsmoothing(cmesh1, paramIn.smothing_angle, tracer);
            break;
        case CGALRepairType::CGAL_TANGENTIAL:
            CGALtangential(cmesh1, tracer);
            break;
        default:
            break;
        }

        //out
        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        if(paramIn.repairType == CGALRepairType::CGAL_BOOLEAN)
            _convertC2T(cmeshOut, *newMesh);
        else
            _convertC2T(cmesh1, *newMesh);
        return newMesh;
    }

    trimesh::TriMesh* subRepairMenu(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
    {
        ////TODO:test error data
        //ErrorInfo info;
        //getErrorInfo(mesh,info);

        if (!mesh)
            return nullptr;

        if (tracer)
        {
            tracer->progress(0.2f);
            if (tracer->interrupt())
                return nullptr;
        }

        //输入
        CMesh cmesh1;
        _convertT2CForNoRepair(*mesh, cmesh1,true);

        if (tracer)
        {
            tracer->progress(0.3f);
            if (tracer->interrupt())
                return nullptr;
        }

        ////自交检测
        std::vector<int> faceIndex;
        if (CGALselfIntersections(cmesh1, faceIndex, true, tracer))
        {
            //孔洞填充
            CGALholeFill(cmesh1, CGALHoleFillType::CGAL_FAIRED, tracer);
        }

        //三角化网格
        CGALtriangulate(cmesh1);

        if (tracer)
        {
            tracer->progress(0.4f);
            if (tracer->interrupt())
                return nullptr;
        }

        //自交检测
        //std::vector<int> faceIndex;
        //自交检测
        //faceIndex.clear();
        //CGALselfIntersections(cmesh1, faceIndex, true, tracer);

        ////非流顶点面检测
        CGALmanifoldness(cmesh1, tracer);    

        if (tracer)
        {
            tracer->progress(0.5f);
            if (tracer->interrupt())
                return nullptr;
        }

        ////重叠面检测
        CGALstitch(cmesh1, tracer);

        if (tracer)
        {
            tracer->progress(0.6f);
            if (tracer->interrupt())
                return nullptr;
        }

        //CGALconnectedComponents(cmesh1, tracer);

        //孔洞填充
        CGALholeFill(cmesh1, CGALHoleFillType::CGAL_TRIANGULATE, tracer);

        if (tracer)
        {
            tracer->progress(0.7f);
            if (tracer->interrupt())
                return nullptr;
        }

        //if (!CGALselfIntersections(cmesh1, faceIndex, true, tracer))
        if(CGAL::is_valid_polygon_mesh(cmesh1))
        {
            //方向修复
            CGALorientation(cmesh1, false, tracer);
        }

        if (tracer)
        {
            tracer->progress(0.8f);
            if (tracer->interrupt())
                return nullptr;
        }

        CGALholeFill(cmesh1, CGALHoleFillType::CGAL_TRIANGULATE, tracer);

        if (tracer)
        {
            tracer->progress(0.9f);
            if (tracer->interrupt())
                return nullptr;
        }

        //输出
        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
         _convertC2T(cmesh1, *newMesh);

         if (tracer)
         {
             tracer->progress(1.0f);
             if (tracer->interrupt())
                 return nullptr;
         }

         //TODO:test error data
         //info.edgeNum = 0;
         //getErrorInfo(newMesh, info);

        return newMesh;
    }
}