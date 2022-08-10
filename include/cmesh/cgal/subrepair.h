#ifndef CMESH_SUBREPAIR_1648026566432_H
#define CMESH_SUBREPAIR_1648026566432_H
#include "cmesh/interface.h"
#include "ccglobal/tracer.h"
#include "trimesh2/TriMesh.h"

namespace cmesh
{
    //���趨�ı߳��ȣ�ϸ���׶��߽�
    CMESH_API trimesh::TriMesh* isotropicRemeshing(trimesh::TriMesh* mesh, double target_edge_length, ccglobal::Tracer* trace = nullptr);

    //ϸ������ģ�͵��ཻ����
    CMESH_API trimesh::TriMesh* corefinement(trimesh::TriMesh* mesh1, trimesh::TriMesh* mesh2, ccglobal::Tracer* trace = nullptr);

    enum class CGALBooleanType
    {
        CGAL_UNION,
        CGAL_INTERSECTION,
        CGAL_DIFFERENCE,
    };
    //boolean
    CMESH_API trimesh::TriMesh* boolean(trimesh::TriMesh* mesh1, trimesh::TriMesh* mesh2, CGALBooleanType type, ccglobal::Tracer* trace = nullptr);

    enum class CGALHoleFillType
    {
        CGAL_TRIANGULATE,
        CGAL_REFINED,
        CGAL_FAIRED,
    };
    //hole fill
    CMESH_API trimesh::TriMesh* holeFill(trimesh::TriMesh* mesh, CGALHoleFillType type, ccglobal::Tracer* trace = nullptr);

    //intersect
    CMESH_API void selfIntersections(trimesh::TriMesh* mesh, std::vector<int>& faceIndex, ccglobal::Tracer* trace = nullptr);

    //orientation
    CMESH_API trimesh::TriMesh* orientation(trimesh::TriMesh* mesh, bool reversible = false, ccglobal::Tracer* trace = nullptr);
}
#endif // CMESH_SUBREPAIR_1648026566432_H