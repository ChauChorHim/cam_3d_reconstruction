#include "pc_tools.h"
#include "files_handler.h"

#include <filesystem>

namespace cch
{

bool loadPcdPath(std::string& pc_folder, std::set<std::string>& pc_list_set) {
    cch::validateFolderDir(pc_folder);

    for (auto &path_to_file: std::filesystem::directory_iterator(pc_folder))
        pc_list_set.insert(path_to_file.path());
    
    return true;
}   

} // namespace cch

