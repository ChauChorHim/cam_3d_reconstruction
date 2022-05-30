#include "files_handler.h"

int main(int argc, char** argv){
    if(argc != 3){
        std::cout << " Wrong number of arguments" << std::endl;
    }

    std::string input_folder_dir(argv[1]);
    std::string output_folder_dir(argv[2]);

    cch::validateFolderDir(input_folder_dir);
    cch::validateFolderDir(output_folder_dir);

    // cch::makeFilesList(input_folder_dir, output_folder_dir + "image_files.txt");

    cch::splitFilesList(
        output_folder_dir + "image_files.txt", 
        output_folder_dir + "train_files.txt", 
        output_folder_dir + "val_files.txt",
        0.9);

    cch::shuffleList(output_folder_dir + "train_files.txt");
    cch::shuffleList(output_folder_dir + "val_files.txt");

    return 0;
}
