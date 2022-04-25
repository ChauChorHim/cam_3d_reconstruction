#ifndef __FILES_HANDLER__
#define __FILES_HANDLER__

#include <string>
#include <iostream>
#include <set>
#include <filesystem>
#include <random>
#include <fstream>
#include <algorithm>
#include <iterator>

#include "npy.h"

namespace cch {

void makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list, bool shuffle, bool with_seq_num, bool keep_folder_dir);

void readline(std::fstream& file, size_t num, std::string& line);

template<class vecType>
void npy2vec(std::string data_fname, std::vector<vecType> &data_out);

/* --------------------------------------------------------------------------------- */

void makeFilesList(const std::string &path_to_files_folder, const std::string &path_to_list, bool shuffle, bool with_seq_num, bool keep_folder_dir) {

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
        file_name = file_name.substr(loc_last_slash+1); // file name with extension
        
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

void readline(std::fstream& file, size_t num, std::string& line) {
    file.seekg(std::ios::beg);
    for (int i = 0; i < num; ++i) {
        file.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    }
    getline(file, line);
}

template<class vecType>
void npy2vec(std::string data_fname, std::vector<vecType> &data_out) {
    std::vector<unsigned long> shape;
    bool is_fortran_order;

    shape.clear();
    data_out.clear();
    npy::LoadArrayFromNumpy(data_fname, shape, is_fortran_order, data_out);

    assert(false == is_fortran_order);
}

};



#endif