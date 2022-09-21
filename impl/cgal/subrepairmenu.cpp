#include "cmesh/cgal/subrepairmenu.h"
#include "subrepair.h"
#include "../cconversion.h"
#include "../ctype.h"
#include "cmesh/uvs/adduvs.h"

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

    void copyUvsInfo(trimesh::TriMesh* dest, trimesh::TriMesh* src)
    {
        if (!dest || !src)
        {
            return;
        }

        dest->UVs = src->UVs;

        dest->faceUVs = src->faceUVs;

        if (dest->faceUVs.size() < dest->faces.size())
        {
            addUVs(dest);
        }

        dest->materials = src->materials;
        dest->mtlName = src->mtlName;
        
        size_t count = sizeof(src->map_bufferSize) / sizeof(int);
        for (size_t i = 0; i < count; i++)
        {
            dest->map_bufferSize[i] = src->map_bufferSize[i];
            dest->map_buffers[i] = src->map_buffers[i];
            src->map_buffers[i] = nullptr;
        }
    }

    bool hasTexture(trimesh::TriMesh* mesh)
    {
        if (mesh)
        {
            size_t count = sizeof(mesh->map_bufferSize) / sizeof(int);
            for (size_t i = 0; i < count; i++)
            {
                if (mesh->map_buffers[i] != nullptr)
                {
                    return true;
                }
            }
        }

        return false;
    }

    trimesh::TriMesh* subMenu(trimesh::TriMesh* mesh, bool refine_and_fair_hole, bool cloudService, ccglobal::Tracer* tracer)
    {
        ////TODO:test error data
//ErrorInfo info;
//getErrorInfo(mesh,info);

       bool hastexture = hasTexture(mesh);

        if (!mesh)
            return nullptr;

        if (tracer)
        {
            tracer->progress(0.2f);
            if (tracer->interrupt())
                return nullptr;
        }

        //ÊäÈë
        CMesh cmesh1;
        _convertT2CForNoRepair(*mesh, cmesh1, cloudService ? false:true);

        if (tracer)
        {
            tracer->progress(0.3f);
            if (tracer->interrupt())
                return nullptr;
        }

        ////×Ô½»¼ì²â
        //std::vector<int> faceIndex;
        //if (CGALselfIntersections(cmesh1, faceIndex, true, tracer))
        //{
        //    //¿×¶´Ìî³ä
        //    CGALholeFill(cmesh1, CGALHoleFillType::CGAL_FAIRED, tracer);
        //}

        //Èý½Ç»¯Íø¸ñ
        if (!hastexture)
            CGALtriangulate(cmesh1);

        if (tracer)
        {
            tracer->progress(0.4f);
            if (tracer->interrupt())
                return nullptr;
        }

        //×Ô½»¼ì²â
        std::vector<int> faceIndex;
        //×Ô½»¼ì²â
        //faceIndex.clear();
        //CGALselfIntersections(cmesh1, faceIndex, true, tracer);

        ////·ÇÁ÷¶¥µãÃæ¼ì²â
        if (!hastexture)
            CGALmanifoldness(cmesh1, tracer);

        if (tracer)
        {
            tracer->progress(0.5f);
            if (tracer->interrupt())
                return nullptr;
        }

        ////ÖØµþÃæ¼ì²â
        if (!hastexture)
            CGALstitch(cmesh1, tracer);

        if (tracer)
        {
            tracer->progress(0.6f);
            if (tracer->interrupt())
                return nullptr;
        }

        //CGALconnectedComponents(cmesh1, tracer);

        //¿×¶´Ìî³ä
        CGALholeFill(cmesh1, refine_and_fair_hole?CGALHoleFillType::CGAL_FAIRED : CGALHoleFillType::CGAL_REFINED, tracer);

        if (tracer)
        {
            tracer->progress(0.7f);
            if (tracer->interrupt())
                return nullptr;
        }

        //if (!CGALselfIntersections(cmesh1, faceIndex, true, tracer))
        if (!hastexture)
            if (CGAL::is_valid_polygon_mesh(cmesh1))
            {
                //·½ÏòÐÞ¸´
                CGALorientation(cmesh1, false, tracer);
            }

        if (tracer)
        {
            tracer->progress(0.8f);
            if (tracer->interrupt())
                return nullptr;
        }

        if (!hastexture)
            CGALholeFill(cmesh1, refine_and_fair_hole ? CGALHoleFillType::CGAL_FAIRED : CGALHoleFillType::CGAL_REFINED, tracer);

        if (tracer)
        {
            tracer->progress(0.9f);
            if (tracer->interrupt())
                return nullptr;
        }

        //Êä³ö
        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh1, *newMesh);

        if (tracer)
        {
            tracer->progress(1.0f);
            if (tracer->interrupt())
                return nullptr;
        }

        if (hastexture)
            copyUvsInfo(newMesh, mesh);
        //TODO:test error data
        //info.edgeNum = 0;
        //getErrorInfo(newMesh, info);

        newMesh->need_bbox();
        return newMesh;
    }

    trimesh::TriMesh* subHoles(trimesh::TriMesh* mesh, bool refine_and_fair_hole, bool cloudService, ccglobal::Tracer* tracer)
    {
        bool hastexture = hasTexture(mesh);

        if (!mesh)
            return nullptr;

        if (tracer)
        {
            tracer->progress(0.2f);
            if (tracer->interrupt())
                return nullptr;
        }

        //ÊäÈë
        CMesh cmesh1;
        _convertT2CForNoRepair(*mesh, cmesh1, cloudService ? false : true);

        if (tracer)
        {
            tracer->progress(0.3f);
            if (tracer->interrupt())
                return nullptr;
        }

        if (tracer)
        {
            tracer->progress(0.4f);
            if (tracer->interrupt())
                return nullptr;
        }

        if (tracer)
        {
            tracer->progress(0.5f);
            if (tracer->interrupt())
                return nullptr;
        }

        if (tracer)
        {
            tracer->progress(0.6f);
            if (tracer->interrupt())
                return nullptr;
        }

        //CGALconnectedComponents(cmesh1, tracer);

        //¿×¶´Ìî³ä
        CGALholeFill(cmesh1, refine_and_fair_hole ? CGALHoleFillType::CGAL_FAIRED : CGALHoleFillType::CGAL_REFINED, tracer);

        if (tracer)
        {
            tracer->progress(0.7f);
            if (tracer->interrupt())
                return nullptr;
        }

        if (tracer)
        {
            tracer->progress(0.8f);
            if (tracer->interrupt())
                return nullptr;
        }

        if (tracer)
        {
            tracer->progress(0.9f);
            if (tracer->interrupt())
                return nullptr;
        }

        //Êä³ö
        trimesh::TriMesh* newMesh = new trimesh::TriMesh;
        _convertC2T(cmesh1, *newMesh);

        if (tracer)
        {
            tracer->progress(1.0f);
            if (tracer->interrupt())
                return nullptr;
        }

        if (hastexture)
            copyUvsInfo(newMesh, mesh);
        //TODO:test error data
        //info.edgeNum = 0;
        //getErrorInfo(newMesh, info);

        newMesh->need_bbox();
        return newMesh;
    }

    trimesh::TriMesh* subRepairMenu(trimesh::TriMesh* mesh, ccglobal::Tracer* tracer)
    {
        return subMenu(mesh, true, false, tracer);
    }

    trimesh::TriMesh* subRepairMenu(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* trace)
    {
        return subMenu(mesh, refine_and_fair_hole, false, trace);
    }

    trimesh::TriMesh* subRepairMenuCloud(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
    {
        return subMenu(mesh, refine_and_fair_hole, true, tracer);
    }

    trimesh::TriMesh* subRepairHolesCloud(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
    {
        return subHoles(mesh, refine_and_fair_hole, true, tracer);
    }
}