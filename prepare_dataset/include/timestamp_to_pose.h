#ifndef _TIMESTAMP_TO_POSE_H_
#define _TIMESTAMP_TO_POSE_H_

#include <eigen3/Eigen/Dense>
#include <string>
#include <vector>

struct GpsData {
    double longitude;
    double latitude;
    double altitude;
    double roll;
    double pitch;
    double yaw;
    std::string timestamp;
};

class Gps2Pose {
private:
    Eigen::Vector3d pos_init_;
    Eigen::Quaterniond q_init_inv_;
    std::vector<GpsData> gps_buffer_;
    int last_search_index_;

    void gps2Pose(GpsData& gps_data, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
    bool compareTimestamp(const std::string& timestamp_1, const std::string& timestamp_2);
    void showGpsData(const GpsData& gps_data);
    void timestamp2Gps(const std::string& timestamp, GpsData& gps_data);

public:
    Gps2Pose();
    void addGpsData(const std::string& path_to_gps_file);
    void initializePose(GpsData *gps_data_init);
    void timestamp2Pose(const std::string& timestamp, Eigen::Vector3d& pos, Eigen::Quaterniond& q);
    void getInitPose(Eigen::Vector3d& pos_init, Eigen::Quaterniond& q_init_inv);
};

#endif