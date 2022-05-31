#ifndef __FILES_HANDLER__
#define __FILES_HANDLER__

#include <string>

namespace cch {

int makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list);

void splitFilesList(
    const std::string &path_to_list, 
    const std::string &path_to_first_list = "./train_files.txt",
    const std::string &path_to_second_list = "./val_files.txt",
    double split_ratio=0.9);

void shuffleList(const std::string &path_to_list);

bool validateFolderDir(std::string &folder_dir);


};

#endif