//
// Created by shaocheng on 9/7/18.
//

#ifndef HACKATHON_VO_UTILS_H
#define HACKATHON_VO_UTILS_H

#include <Eigen/Dense>
#include <opencv2/core/core.hpp>
#include "features.h"

class Utils {
public:
    static double getAbsoluteScale(const std::vector<double> &scales, int index);

    static std::vector<std::vector<double>> computeScales(const std::string &in_file_name);

};

Eigen::Vector3d point3dToEigen(const cv::Point3d &point);

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matToEigen(const cv::Mat &matrix);

cv::Point3d eigenToPoint3d(const Eigen::Vector3d &vector);

cv::Mat eigenToMat(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &matrix);

std::vector<std::vector<Eigen::Vector2d>> GetLast3TrackFeatures(const std::vector<cv::Point2f> &points_1_detect,
                                                                const std::vector<cv::Point2f> &points_2_matched,
                                                                const std::vector<cv::Point2f> &points_3_matched2,
                                                                const std::vector<uchar> &status_matched2);
#endif //HACKATHON_VO_UTILS_H GetLast3TrackFeatures
