#include "cmesh/igl/str.h"
#include "igl/string_utils.h"

namespace cmesh
{
	bool starts_with(const std::string& str, const std::string& prefix)
	{
		return igl::starts_with(str, prefix);
	}
}