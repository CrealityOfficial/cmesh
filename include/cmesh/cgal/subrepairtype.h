#ifndef CMESH_SUBREPAIRTYPE_1648026566432_H
#define CMESH_SUBREPAIRTYPE_1648026566432_H
#include <vector>

namespace cmesh
{
    enum class CGALRepairType  //�޸�����
    {
        CGAL_ISOTROPIC,
        CGAL_COREFINE,
        CGAL_BOOLEAN,
        CGAL_SELFINTERSECT,
        CGAL_ORIENTATION,
        CGAL_STITCH,
        CGAL_MANIFOLDNESS,
        CGAL_CONNECTED,
        CGAL_HOLE,
        CGAL_TRIANGULATE,
        CGAL_FEATURES
    };

    enum class CGALBooleanType  //������������
    {
        CGAL_UNION,
        CGAL_INTERSECTION,
        CGAL_DIFFERENCE,
    };

    enum class CGALHoleFillType  //�׶��޸�����
    {
        CGAL_TRIANGULATE,
        CGAL_REFINED,
        CGAL_FAIRED,
    };

    struct RepairMenuParamIn
    {
        CGALRepairType repairType;  //�޸�����

        double target_edge_length;  //ϸ���׶��߽� �߳�   
        CGALBooleanType booleanType; //������������
        CGALHoleFillType holeFillType; //�׶��������
        bool reversible;    //�����޸� �Ƿ����޸�
        std::vector<int> faceIndex;  //�洢���ཻ����

        RepairMenuParamIn()
        {
            target_edge_length = 0.1f;
            repairType = CGALRepairType::CGAL_ISOTROPIC;
            booleanType = CGALBooleanType::CGAL_UNION;
            holeFillType = CGALHoleFillType::CGAL_TRIANGULATE;
            reversible = false;
            faceIndex.clear();
        }
    };
}
#endif // CMESH_SUBREPAIRTYPE_1648026566432_H