#ifndef CMESH_SUBREPAIRTYPE_1648026566432_H
#define CMESH_SUBREPAIRTYPE_1648026566432_H
#include <vector>

namespace cmesh
{
    enum class CGALRepairType  //修复类型
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

    enum class CGALBooleanType  //布尔运算类型
    {
        CGAL_UNION,
        CGAL_INTERSECTION,
        CGAL_DIFFERENCE,
    };

    enum class CGALHoleFillType  //孔洞修复类型
    {
        CGAL_TRIANGULATE,
        CGAL_REFINED,
        CGAL_FAIRED,
    };

    struct RepairMenuParamIn
    {
        CGALRepairType repairType;  //修复类型

        double target_edge_length;  //细化孔洞边界 边长   
        CGALBooleanType booleanType; //布尔运算类型
        CGALHoleFillType holeFillType; //孔洞填充类型
        bool reversible;    //方向修复 是否反向修复
        std::vector<int> faceIndex;  //存储自相交的面

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