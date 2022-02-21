#ifndef _UNDISTORT_H
#define _UNDISTORT_H

#include <opencv2/core/core.hpp>
#include <vector>
#include <string>

#include <filesystem>
#include <fstream>
#include <iterator>
#include <iostream>
#include <iomanip>
#include <algorithm>
#include <random>

#include <unordered_set>
#include <set>

#include <cstdio>

class ImagesHandler {
    private:
        cv::Mat camera_matrix_;
        cv::Mat dist_coeffs_;
        std::vector<std::string> images_path_buffer_;
    
    public:
        ImagesHandler();
        void readCameraParameters(const std::string &yml_filename);
        void loadImages(const std::string &input_folder_dir, const std::string &images_list);
        void undistortImages(const std::string &output_folder_dir);
        void cropImages(cv::Range &row_range, cv::Range &col_range, const std::string &output_folder_dir);
        void resizeImages(int height, int width, const std::string &output_folder_dir);
};

class FilesHandler {
    public:
        static void makeFilesList(
            const std::string &path_to_files_folder, 
            const std::string &path_to_list="./files_list.txt", 
            bool shuffle=false) {
                std::cout << "-> The path to files list is: " << path_to_list << std::endl;
                std::vector<std::string> files_list = {};
                int idx = 0;

                std::set<std::filesystem::path> sorted_by_name;
                for (const auto &path_to_file: std::filesystem::directory_iterator(path_to_files_folder))
                    sorted_by_name.insert(path_to_file.path());

                for (const auto& path_to_file : sorted_by_name) {
                    std::string file_list (path_to_file);

                    int loc_last_slash = file_list.find_last_of('/');
                    file_list = file_list.substr(loc_last_slash+1, file_list.size()-1);

                    files_list.push_back(std::to_string(idx++) + " " + file_list);
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

        static void splitFilesList(const std::string &path_to_list, int train_dataset_ratio=0.9) {
            int number_of_lines = 0;
            std::string line;
            std::ifstream original_list(path_to_list);
            while (std::getline(original_list, line))
                ++number_of_lines;

            int loc_last_slash = path_to_list.find_last_of('/');
            std::string directory_to_list = path_to_list.substr(0, loc_last_slash+1);
            std::string path_to_train_list = directory_to_list + "train_files.txt";
            std::string path_to_val_list = directory_to_list + "val_files.txt";

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

        // TODO add "path_to_output_folder" and a button "is_the_same_folder"
        static void keepFirstNFiles(const std::string &path_to_files_folder, int N) {
            int files_num = std::distance(std::filesystem::directory_iterator(path_to_files_folder),
                std::filesystem::directory_iterator());
            std::unordered_set<int> indexes = {};
            for (int i = N; i < files_num; ++i) 
                indexes.insert(i);
            deleteFiles(path_to_files_folder, indexes);
        }

        static void renameFiles(const std::string &path_to_files_folder, int type = 1) {
            std::set<std::filesystem::path> sorted_by_name;
            for (auto &entry: std::filesystem::directory_iterator(path_to_files_folder))
                sorted_by_name.insert(entry.path());

            if (type == 1) {
                int index = 0;
                for (auto &path_to_file : sorted_by_name) {
                    std::rename(path_to_file.c_str(), (path_to_files_folder + "/" + std::to_string(index++) + ".jpg").c_str());
                } 
            } else {
                std::cout << "Undefined naming strategy\n";
            }
            return;
        }

    private:
        static void deleteFiles(const std::string &path_to_files_folder, const std::unordered_set<int> &indexes) {
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
};


#endif