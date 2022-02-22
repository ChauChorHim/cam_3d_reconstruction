#include "files_handler.h"

#include <iostream>
#include <set>
#include <filesystem>
#include <random>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <cstdio>

void FilesHandler::makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list, bool shuffle, bool rename_files) {

    std::cout << "-> The path to files list is: " << path_to_list << std::endl;
    if (std::filesystem::exists(path_to_list)) {
        std::remove(path_to_list.c_str());
    }

    if (rename_files) {  // strongly recommend to do so
        FilesHandler::renameFiles(path_to_files_folder);
    }
    
    std::set<std::filesystem::path> sorted_by_name;

    for (const auto &path_to_file: std::filesystem::directory_iterator(path_to_files_folder))
        sorted_by_name.insert(path_to_file.path());

    
    std::vector<std::string> files_list = {};
    int idx = 0;

    for (const auto& path_to_file : sorted_by_name) {
        std::string file_list (path_to_file);

        int loc_last_slash = file_list.find_last_of('/');
        file_list = file_list.substr(loc_last_slash+1, file_list.size()-1);

        files_list.push_back(std::to_string(idx++) + " " + file_list);  // only get index + the file name 
    }

    if(shuffle) {
        auto rd = std::random_device {};
        auto rng = std::default_random_engine {rd()};
        std::shuffle(std::begin(files_list), std::end(files_list), rng);
    }

    std::ofstream list_file(path_to_list);
    std::ostream_iterator<std::string> list_iterator(list_file, "\n");
    std::copy(files_list.begin(), files_list.end(), list_iterator);
}

void FilesHandler::splitFilesList(const std::string &path_to_list, int train_dataset_ratio) {
    int number_of_lines = 0;
    std::string line;
    std::ifstream original_list(path_to_list);
    while (std::getline(original_list, line))
        ++number_of_lines;

    int loc_last_slash = path_to_list.find_last_of('/');
    std::string directory_to_list = path_to_list.substr(0, loc_last_slash+1);
    std::string path_to_train_list = directory_to_list + "train_files.txt";
    std::string path_to_val_list = directory_to_list + "val_files.txt";

    if (std::filesystem::exists(path_to_train_list)) {
        std::remove(path_to_train_list.c_str());
    }

    if (std::filesystem::exists(path_to_val_list)) {
        std::remove(path_to_val_list.c_str());
    }

    std::ifstream input_list(path_to_list);
    std::ofstream train_files(path_to_train_list);
    std::ofstream val_files(path_to_val_list);

    int count = 0;
    while (std::getline(input_list, line)) {
        // std::cout << line << std::endl;
        if (count < 0.9 * number_of_lines) {
            train_files << line << std::endl;
        } else {
            val_files << line << std::endl;
        }
        count++;
    }
}

void FilesHandler::keepFirstNFiles(const std::string &path_to_files_folder, int N) {
    int files_num = std::distance(std::filesystem::directory_iterator(path_to_files_folder),
        std::filesystem::directory_iterator());
    std::unordered_set<int> indexes = {};
    for (int i = N; i < files_num; ++i) 
        indexes.insert(i);
    deleteFiles(path_to_files_folder, indexes);
}




void FilesHandler::renameFiles(const std::string &path_to_files_folder) {
    
    std::set<std::filesystem::path> sorted_by_name;

    for (auto &entry: std::filesystem::directory_iterator(path_to_files_folder))
        sorted_by_name.insert(entry.path());

    int index = 0;
    for (auto &path_to_file : sorted_by_name) {
        std::stringstream ss;
        ss << std::setw(6) << std::setfill('0') << index++ << ".jpg";
        std::string new_file_name = ss.str();
        std::rename(path_to_file.c_str(), (path_to_files_folder + "/" + new_file_name).c_str());
    } 
    return;
}

void FilesHandler::deleteFiles(const std::string &path_to_files_folder, const std::unordered_set<int> &indexes) {
    auto iter = std::filesystem::directory_iterator(path_to_files_folder);
    std::set<std::filesystem::path> sorted_by_name;
    for (auto &entry: std::filesystem::directory_iterator(path_to_files_folder))
        sorted_by_name.insert(entry.path());
    int idx = 0;
    for (auto path_to_cur_file : sorted_by_name) {
        if (indexes.find(idx) != indexes.end()) {
            std::filesystem::remove(path_to_cur_file);
        }
        idx++;
    }
}