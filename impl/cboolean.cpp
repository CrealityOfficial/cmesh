#include "cboolean.h"

namespace cmesh
{
	bool _boolean(CMesh& mesh1, CMesh& mesh2, CMesh& out_union, CMesh& out_intersection)
	{
		if (!CGAL::Polygon_mesh_processing::does_self_intersect(mesh1)
			&& !CGAL::Polygon_mesh_processing::does_self_intersect(mesh2)
			&& CGAL::Polygon_mesh_processing::does_bound_a_volume(mesh1)
			&& CGAL::Polygon_mesh_processing::does_bound_a_volume(mesh2))
		{

			std::array<boost::optional<CMesh*>, 4> output;
			output[PMP::Corefinement::UNION] = &out_union;
			output[PMP::Corefinement::INTERSECTION] = &out_intersection;
			// for the example, we explicit the named parameters, this is identical to
			// PMP::corefine_and_compute_boolean_operations(mesh1, mesh2, output)

			std::array<bool, 4> res =
				PMP::corefine_and_compute_boolean_operations(
					mesh1, mesh2,
					output,
					params::all_default(), // mesh1 named parameters
					params::all_default(), // mesh2 named parameters
					std::make_tuple(
						params::vertex_point_map(get(boost::vertex_point, out_union)), // named parameters for out_union
						params::vertex_point_map(get(boost::vertex_point, out_intersection)), // named parameters for out_intersection
						params::all_default(), // named parameters for mesh1-mesh2 not used
						params::all_default())// named parameters for mesh2-mesh1 not used)
				);
			return true;
		}
		else
			return false;
	}
}