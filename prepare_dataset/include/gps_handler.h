#ifndef _GPS_HANDLER_H_
#define _GPS_HANDLER_H_

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
};

#endif