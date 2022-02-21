#include "files_handler.h"
#include <iostream>
#include <string>
#include <cstdlib>

void printHelp(char* programName){
    std::cout << " Correct usage: " << programName << " input_folder_dir" << std::endl;
    exit(EXIT_FAILURE);
}

int main(int argc, char** argv){
    if(argc != 2){
        std::cout << " Wrong number of arguments" << std::endl;
        printHelp(argv[0]);
    }

    FilesHandler filesHandler = FilesHandler();
    std::string input_folder_dir(argv[1]);

    filesHandler.renameFiles(input_folder_dir);

    return 0;
}