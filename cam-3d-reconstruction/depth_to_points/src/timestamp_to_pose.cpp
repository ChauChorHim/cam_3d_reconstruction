#include "timestamp_to_pose.h"
#include "gps2utm.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <iomanip>

Gps2Pose::Gps2Pose() : last_search_index_(0){
}

void Gps2Pose::addGpsData(const std::string& path_to_gps_file) {
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

void Gps2Pose::showGpsData(const GpsData& gps_data) {
    std::cout << std::setprecision(18)
              << "timestamp: " << gps_data.timestamp << " "
              << "Latitude: " << gps_data.latitude << " "
              << "Longitude: " << gps_data.longitude << " "
              << "Altitude: " << gps_data.altitude << " "
              << "Roll: " << gps_data.roll << " "
              << "Ptich: " << gps_data.pitch << " "
              << "Yaw: " << gps_data.yaw <<  "\n";
}


void Gps2Pose::initializePose(GpsData *gps_data_init) {
    if(gps_data_init == nullptr)
        gps_data_init = &gps_buffer_[0];

    double roll = gps_data_init->roll;
    double pitch = -gps_data_init->pitch;
    double yaw = M_PI / 2. - gps_data_init->yaw * DEG_TO_RAD;

    Eigen::Quaterniond q_init = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    q_init_inv_ = q_init.inverse(); 

    double utm_east_init = 0.;
    double utm_north_init = 0.;
    double utm_up_init = 0.;
    aw::coord::GPS2UTM(gps_data_init->latitude,
                       gps_data_init->longitude, 
                       gps_data_init->altitude,
                       &utm_east_init, 
                       &utm_north_init, 
                       &utm_up_init);

    pos_init_ = Eigen::Vector3d(utm_east_init, utm_north_init, utm_up_init);
}
    
void Gps2Pose::gps2Pose(GpsData& gps_data, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {

    double roll = gps_data.roll;
    double pitch = -gps_data.pitch;
    double yaw = M_PI / 2 - gps_data.yaw * DEG_TO_RAD;
    Eigen::Quaterniond q_tmp = Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) * 
                           Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());

    q = q_init_inv_ * q_tmp;

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

    pos = q_init_inv_.toRotationMatrix() * (pos - pos_init_);
}

// If the first timestamp is smaller than the second one, than return true
bool Gps2Pose::compareTimestamp(const std::string& timestamp_1, const std::string& timestamp_2) {
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
void Gps2Pose::timestamp2Gps(const std::string& timestamp, GpsData& gps_data) {
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


void Gps2Pose::timestamp2Pose(const std::string& timestamp, Eigen::Vector3d& pos, Eigen::Quaterniond& q) {
    GpsData gps_data;
    timestamp2Gps(timestamp, gps_data);
    gps2Pose(gps_data, pos, q);
}
    
void Gps2Pose::getInitPose(Eigen::Vector3d& pos_init, Eigen::Quaterniond& q_init_inv) {
    pos_init = pos_init_;
    q_init_inv = q_init_inv_;
}