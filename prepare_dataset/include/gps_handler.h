#ifndef _GPS_HANDLER_H_
#define _GPS_HANDLER_H_

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>
#include "gps2utm.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>

namespace cch {

struct GpsData {
    double longitude;
    double latitude;
    double altitude;
    double roll;
    double pitch;
    double yaw;
    std::string timestamp;
};

class GpsHandler {
private:
    std::vector<GpsData> gps_buffer_;
    int last_search_index_;

    void cacheGpsData(const std::string& path_to_gps_file);
    bool compareTimestamp(const std::string& timestamp_1, const std::string& timestamp_2);

public:
    GpsHandler(const std::string& path_to_gps_file);
    void timestamp2Gps(const std::string& timestamp, GpsData& gps_data);
    void gps2UtmPose(GpsData& gps_data, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
    void showGpsData(const GpsData& gps_data);
    void loopGpsBuffer();
};

/* --------------------------------------------------------------------------------- */

GpsHandler::GpsHandler(const std::string& path_to_gps_file) : last_search_index_(0){
    cacheGpsData(path_to_gps_file);
}

void GpsHandler::cacheGpsData(const std::string& path_to_gps_file) {
    std::ifstream gps_file (path_to_gps_file);
    if (!gps_file.is_open()) {
        std::cerr << "Could not open the file - '"
                  << path_to_gps_file << "'" << std::endl;
        exit(EXIT_FAILURE);
    }
    std::string line;
    GpsData gps_data;

    bool is_remove_header = false;
    
    while(std::getline(gps_file, line)) {
        if (!is_remove_header) {
            std::getline(gps_file, line);
            is_remove_header = true;
        }
        std::vector<int> comma_location;
        for (size_t i = 0; i < line.size(); ++i) {
            if (line[i] == ',') {
                comma_location.push_back(i);
            }
        }
        int start = 0, end = 0;

        start = 0;
        end = comma_location[0];
        gps_data.timestamp = line.substr(start, end - start);

        start = comma_location[4];
        end = comma_location[5];
        gps_data.latitude = std::atof(line.substr(start + 1, end - start).c_str());
        
        start = comma_location[5];
        end = comma_location[6];
        gps_data.longitude = std::stold(line.substr(start + 1, end - start));

        start = comma_location[6];
        end = comma_location[7];
        gps_data.altitude = std::stold(line.substr(start + 1, end - start));
        
        start = comma_location[16];
        end = comma_location[17];
        gps_data.roll = std::stold(line.substr(start + 1, end - start));

        start = comma_location[17];
        end = comma_location[18];
        gps_data.pitch = std::stold(line.substr(start + 1, end - start));

        start = comma_location[18];
        end = comma_location[19];
        gps_data.yaw = std::stold(line.substr(start + 1, end - start));

        // showGpsData(gps_data);
        gps_buffer_.push_back(gps_data);
    }

    std::cout << " --- gps_buffer_.size() = " << gps_buffer_.size() << "\n";

}

void GpsHandler::showGpsData(const GpsData& gps_data) {
    std::cout << std::setprecision(18)
              << "timestamp: " << gps_data.timestamp << " "
              << "Latitude: " << gps_data.latitude << " "
              << "Longitude: " << gps_data.longitude << " "
              << "Altitude: " << gps_data.altitude << " "
              << "Roll: " << gps_data.roll << " "
              << "Ptich: " << gps_data.pitch << " "
              << "Yaw: " << gps_data.yaw <<  "\n";
}

void GpsHandler::gps2UtmPose(GpsData& gps_data, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    // double roll = gps_data.roll;
    // double pitch = gps_data.pitch;
    // double yaw = gps_data.yaw * DEG_TO_RAD;
    double roll = gps_data.roll * DEG_TO_RAD;
    double pitch = -gps_data.pitch * DEG_TO_RAD;
    double yaw = M_PI / 2. - gps_data.yaw * DEG_TO_RAD;

    q = Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()) * 
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());

    double utm_east = 0.;
    double utm_north = 0.;
    double utm_up = 0.;
    aw::coord::GPS2UTM(gps_data.latitude,
                       gps_data.longitude, 
                       gps_data.altitude,
                       &utm_east, 
                       &utm_north, 
                       &utm_up);
    pos = Eigen::Vector3d(utm_east, utm_north, utm_up);
}
    

// If the first timestamp is smaller than the second one, than return true
bool GpsHandler::compareTimestamp(const std::string& timestamp_1, const std::string& timestamp_2) {
    std::string first_part_1 = timestamp_1.substr(0, timestamp_1.find('.'));
    std::string first_part_2 = timestamp_2.substr(0, timestamp_2.find('.'));

    if (first_part_1 < first_part_2) return true;
    if (first_part_1 > first_part_2) return false;

    std::string second_part_1 = timestamp_1.substr(timestamp_1.find('.') + 1);
    std::string second_part_2 = timestamp_2.substr(timestamp_2.find('.') + 1);

    if (second_part_1 < second_part_2) return true;
    else return false;

}


// Find the nearest timestamp gps data
void GpsHandler::timestamp2Gps(const std::string& timestamp, GpsData& gps_data) {
    if (compareTimestamp(timestamp, gps_buffer_[last_search_index_].timestamp)) {
        while (last_search_index_ >= 0 && compareTimestamp(timestamp, gps_buffer_[last_search_index_].timestamp)) {
            last_search_index_--;
        }
    } else {
        while (last_search_index_ < gps_buffer_.size() && !compareTimestamp(timestamp, gps_buffer_[last_search_index_].timestamp)) {
            last_search_index_++;
        }
    }
    gps_data = gps_buffer_[last_search_index_]; 

    gps_data.timestamp = timestamp;
}


void GpsHandler::loopGpsBuffer() {

    std::vector<double> rolls;
    std::vector<double> pitchs;
    std::vector<double> yaws;

    for (auto &ele : gps_buffer_) {
        rolls.push_back(ele.roll);
        pitchs.push_back(ele.pitch);
        yaws.push_back(ele.yaw);
    }

    std::cout << *std::max_element(rolls.begin(), rolls.end()) << " " << *std::min_element(rolls.begin(), rolls.end()) << "\n";
    std::cout << *std::max_element(pitchs.begin(), pitchs.end()) << " " << *std::min_element(pitchs.begin(), pitchs.end()) << "\n";
    std::cout << *std::max_element(yaws.begin(), yaws.end()) << " " << *std::min_element(yaws.begin(), yaws.end()) << "\n";
}

#endif

};