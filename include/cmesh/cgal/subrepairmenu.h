#ifndef CMESH_SUBREPAIRMENU_1648026566432_H
#define CMESH_SUBREPAIRMENU_1648026566432_H
#include "cmesh/interface.h"
#include "ccglobal/tracer.h"
#include "trimesh2/TriMesh.h"
#include "subrepairtype.h"

namespace cmesh
{
    CMESH_API trimesh::TriMesh* subRepairMenu(trimesh::TriMesh* mesh, trimesh::TriMesh* mesh2,RepairMenuParamIn& paramIn, ccglobal::Tracer* trace = nullptr);

    CMESH_API trimesh::TriMesh* subRepairMenu(trimesh::TriMesh* mesh, ccglobal::Tracer* trace = nullptr);

    CMESH_API trimesh::TriMesh* subRepairMenu(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* trace = nullptr);

    CMESH_API trimesh::TriMesh* subRepairMenuCloud(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* trace = nullptr);
   
    CMESH_API trimesh::TriMesh* subRepairHolesCloud(trimesh::TriMesh* mesh, bool refine_and_fair_hole, ccglobal::Tracer* trace = nullptr);
}
#endif // CMESH_SUBREPAIRMENU_1648026566432_H