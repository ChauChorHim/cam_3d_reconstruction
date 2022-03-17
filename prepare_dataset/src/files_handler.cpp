#include "files_handler.h"

#include <iostream>
#include <set>
#include <filesystem>
#include <random>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <cstdio>
#include <cassert>

void FilesHandler::makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list, bool shuffle, bool with_seq_num, bool keep_folder_dir, bool keep_extension) {

    std::cout << "-> The path to files list is: " << path_to_list << std::endl;
    if (std::filesystem::exists(path_to_list)) {
        std::remove(path_to_list.c_str());
    }

    std::set<std::filesystem::path> sorted_by_name;

    for (const auto &path_to_file: std::filesystem::directory_iterator(path_to_files_folder))
        sorted_by_name.insert(path_to_file.path());

    
    std::vector<std::string> files_list = {};
    int idx = 0;

    for (const auto& path_to_file : sorted_by_name) {
        std::string file_name (path_to_file);

        int loc_last_slash = file_name.find_last_of('/');
        if (keep_extension) {
            file_name = file_name.substr(loc_last_slash+1); // file name with extension
        } else {
            int loc_last_dot = file_name.find_last_of('.');
            file_name = file_name.substr(loc_last_slash+1, loc_last_dot - (loc_last_slash+1)); // file name without extension (aka timestamp)
        }
        

        std::string file_list;
        if (keep_folder_dir) {
            file_list = path_to_files_folder + file_name;
        } else {
            file_list = file_name;
        }

        if (with_seq_num) {
            file_list = std::to_string(idx++) + " " + file_list;
        } else {
            file_list = file_list;
        }
        files_list.push_back(file_list); 
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

void FilesHandler::removeMarginList(std::vector<std::string> &files_list, std::vector<int>& margin_list) {
    // TODO erase a vector from a vector
    files_list.erase(files_list.begin());
    files_list.erase(files_list.begin());
    files_list.erase(files_list.begin());
    files_list.erase(files_list.end());
    files_list.erase(files_list.end());
}



void FilesHandler::splitFilesList(const std::string &path_to_list, double train_dataset_ratio, bool shuffle, bool remove_margin) {
    std::vector<std::string> path_to_files;
    std::string path_to_file;

    std::ifstream list(path_to_list);
    while (std::getline(list, path_to_file)) {
        path_to_files.push_back(path_to_file);
    }

    if(remove_margin) {
        std::vector<int> margin_list {0, 1, 2, -1, -2};
        FilesHandler::removeMarginList(path_to_files, margin_list);
    }

    std::vector<std::string> path_to_train_files, path_to_val_files; 

    for (int count = 0; count < path_to_files.size(); ++count) {
        // std::cout << line << std::endl;
        if (count < train_dataset_ratio * path_to_files.size()) {
            path_to_train_files.push_back(path_to_files[count]);
        } else {
            path_to_val_files.push_back(path_to_files[count]);
        }
    }

    if (shuffle) {
        auto rd = std::random_device {};
        auto rng = std::default_random_engine {rd()};
        std::shuffle(std::begin(path_to_train_files), std::end(path_to_train_files), rng);
        std::shuffle(std::begin(path_to_val_files), std::end(path_to_val_files), rng);
    }

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

    std::ofstream train_files(path_to_train_list);
    std::ofstream val_files(path_to_val_list);

    std::ostream_iterator<std::string> train_list_iterator(train_files, "\n");
    std::copy(path_to_train_files.begin(), path_to_train_files.end(), train_list_iterator);

    std::ostream_iterator<std::string> val_list_iterator(val_files, "\n");
    std::copy(path_to_val_files.begin(), path_to_val_files.end(), val_list_iterator);

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