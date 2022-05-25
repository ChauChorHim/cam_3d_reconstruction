#ifndef __FILES_HANDLER__
#define __FILES_HANDLER__

#include <string>
#include <vector>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <set>
#include <random>
#include <iterator>
#include <algorithm>

namespace cch {

int makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list);

void splitFilesList(
    const std::string &path_to_list, 
    const std::string &path_to_first_list = "./train_files.txt",
    const std::string &path_to_second_list = "./val_files.txt",
    double split_ratio=0.9);

void shuffleList(const std::string &path_to_list);

bool validateFolderDir(std::string &folder_dir);

/* --------------------------------------------------------------------------------- */

int makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list) {

    std::cout << "-> The path to files list is: " << path_to_list << std::endl;

    std::set<std::filesystem::path> sorted_by_name;

    for (const auto &path_to_file: std::filesystem::directory_iterator(path_to_files_folder))
        sorted_by_name.insert(path_to_file.path());

    std::vector<std::string> files_list = {};
    std::string file_list {};
    int idx = 0;

    for (const auto& path_to_file : sorted_by_name) {
        std::string file_name = path_to_file.filename();

        file_list = std::to_string(idx++) + " " + file_name;
        
        files_list.push_back(file_list); 
    }

    if (std::filesystem::exists(path_to_list)) {
        std::remove(path_to_list.c_str());
    }
    std::ofstream list_file(path_to_list);
    std::ostream_iterator<std::string> list_iterator(list_file, "\n");
    std::copy(files_list.begin()+2, files_list.end()-2, list_iterator);

    return idx;
}

void splitFilesList(
    const std::string &path_to_list, 
    const std::string &path_to_first_list,
    const std::string &path_to_second_list,
    double split_ratio) {

    std::string cur_line;

    size_t number_of_lines = 0;
    std::ifstream list_file_count(path_to_list);
    while (std::getline(list_file_count, cur_line))
        ++number_of_lines;

    std::ifstream list_file(path_to_list);
    std::vector<std::string> first_lines, second_lines;
    size_t index = 0;
    while (std::getline(list_file, cur_line)) {
        if (index++ < 0.9 * number_of_lines)
            first_lines.push_back(cur_line);
        else 
            second_lines.push_back(cur_line);
    }

    if (std::filesystem::exists(path_to_first_list)) {
        std::remove(path_to_first_list.c_str());
    }
    if (std::filesystem::exists(path_to_second_list)) {
        std::remove(path_to_second_list.c_str());
    }

    std::ofstream first_files(path_to_first_list);
    std::ofstream second_files(path_to_second_list);

    std::ostream_iterator<std::string> first_list_iterator(first_files, "\n");
    std::copy(first_lines.begin(), first_lines.end(), first_list_iterator);

    std::ostream_iterator<std::string> second_list_iterator(second_files, "\n");
    std::copy(second_lines.begin(), second_lines.end(), second_list_iterator);
}

void shuffleList(const std::string &path_to_list) {

    std::string cur_line;
    std::ifstream list_file(path_to_list);
    std::vector<std::string> lines;

    while (std::getline(list_file, cur_line)) {
        lines.push_back(cur_line);
    }

    auto rd = std::random_device {};
    auto rng = std::default_random_engine {rd()};
    std::shuffle(std::begin(lines), std::end(lines), rng);

    std::ofstream list_file_(path_to_list);

    std::ostream_iterator<std::string> list_file_iterator(list_file_, "\n");
    std::copy(lines.begin(), lines.end(), list_file_iterator);
}


bool validateFolderDir(std::string &folder_dir) {
    std::filesystem::path folder_path(folder_dir);
    if (!std::filesystem::exists(folder_path)) { 
        std::cout << "\nError: check folder_dir exists: " + folder_dir + "\n";
        return false;
    }
    
    std::filesystem::directory_entry folder_entry(folder_dir);
    if (folder_entry.status().type() != std::filesystem::file_type::directory) {
        std::cout << "\nError: this is not a direcotry\n";
        return false;
    }

    if (folder_dir.back() != '/') {
        folder_dir.push_back('/');
    }

    return true;
}

};

#endif