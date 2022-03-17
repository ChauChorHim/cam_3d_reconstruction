#ifndef __FILES_HANDLER__
#define __FILES_HANDLER__

#include <string>
#include <vector>
#include <unordered_set>

class FilesHandler {
public:
    void makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list="./files_list.txt", bool shuffle=true, bool with_seq_num=true, bool keep_folder_dir=false, bool keep_extension=true);


    void splitFilesList(const std::string &path_to_list, double train_dataset_ratio=0.9, bool shuffle=true, bool remove_margin=true);
        
    // TODO add "path_to_output_folder" and a button "is_the_same_folder"
    void keepFirstNFiles(const std::string &path_to_files_folder, int N);

private:
    void renameFiles(const std::string &path_to_files_folder);
    void deleteFiles(const std::string &path_to_files_folder, const std::unordered_set<int> &indexes);
    void removeMarginList(std::vector<std::string> &files_list, std::vector<int> &margin_list);
};

#endif