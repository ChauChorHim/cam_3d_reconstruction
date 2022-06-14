#ifndef __PC_TOOLS_H
#define __PC_TOOLS_H

#include <cstring>
#include <set>

namespace cch
{

bool loadPcdPath(std::string& pc_folder, std::set<std::string>& pc_list_set);

} // namespace cch


#endif