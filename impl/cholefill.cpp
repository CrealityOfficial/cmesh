#include "cholefill.h"

namespace cmesh
{
    bool _holeFilling(CMesh& cmesh, bool refine_and_fair_hole, ccglobal::Tracer* tracer)
    {
        if (tracer)
        {
            tracer->progress(0.4f);
        }

        // Both of these must be positive in order to be considered
        double max_hole_diam = 0.1;
        int max_num_hole_edges = 1;

        std::vector<halfedge_descriptor> border_cycles;
        // collect one halfedge per boundary cycle
        PMP::extract_boundary_cycles(cmesh, std::back_inserter(border_cycles));

        float countNums = border_cycles.size() == 0 ? 1 : 1.0 / border_cycles.size();
        int count = 1;

        for (halfedge_descriptor h : border_cycles)
        {
            if (tracer)
            {
                tracer->progress(countNums * count++);
            }

            std::vector<face_descriptor>  patch_facets;
            std::vector<vertex_descriptor> patch_vertices;
            if (refine_and_fair_hole)
            {

                bool success = std::get<0>(PMP::triangulate_refine_and_fair_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices)));
            }
            else
            {
                PMP::triangulate_and_refine_hole(cmesh,
                    h,
                    std::back_inserter(patch_facets),
                    std::back_inserter(patch_vertices));
            }
        }

        if (tracer)
        {
            tracer->progress(1.0f);
        }
        return true;
    }
}