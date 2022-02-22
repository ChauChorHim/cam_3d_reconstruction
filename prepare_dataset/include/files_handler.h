#include <string>
#include <unordered_set>

class FilesHandler {
    public:
        void makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list="./files_list.txt", bool shuffle=false, bool rename_files=true);

        void splitFilesList(const std::string &path_to_list, int train_dataset_ratio=0.9);
            
        // TODO add "path_to_output_folder" and a button "is_the_same_folder"
        void keepFirstNFiles(const std::string &path_to_files_folder, int N);

    private:
        void renameFiles(const std::string &path_to_files_folder);
        void deleteFiles(const std::string &path_to_files_folder, const std::unordered_set<int> &indexes);
};