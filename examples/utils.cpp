//
// Created by shaocheng on 9/7/18.
//

#include "utils.h"
double Utils::getAbsoluteScale(const std::vector<double> &scales, int index) {
    return scales[index];
}

std::vector<std::vector<double> > Utils::computeScales(const std::string &in_file_name) {
    std::ifstream myfile(in_file_name.c_str());


    std::vector<std::vector<double> > res;
    res.resize(3);

    if (!myfile.is_open()) {
        std::cout << "Unable to open file";
        return res;
    }

    std::string line;
    int i = 0;
    double x_curr = 0, y_curr = 0, z_curr = 0;
    double x_prev, y_prev, z_prev;

    while (getline(myfile, line)) {
        z_prev = z_curr;
        x_prev = x_curr;
        y_prev = y_curr;
        std::istringstream in(line);
        for (int j = 0; j < 12; j++) {
            in >> z_curr;
            if (j == 7) y_curr = z_curr;
            if (j == 3) x_curr = z_curr;
        }

        if (i == 0) {
            res[0].push_back(0);
            res[1].push_back(0);
            res[2].push_back(0);
            i++;
            continue;
        }
        double scale = sqrt((x_curr - x_prev) * (x_curr - x_prev)
                            + (y_curr - y_prev) * (y_curr - y_prev)
                            + (z_curr - z_prev) * (z_curr - z_prev));

        if (i == 1) {
            res[0].push_back(scale);
            res[1].push_back(1);
            res[2].push_back(1);
            i++;
            continue;
        }
        res[0].push_back(scale); // absolute scale in m
        res[1].push_back(scale / res[0][1]); // relative scale wrt the first scale
        res[2].push_back(scale / res[0][i - 1]); // relative scale wrt the previous scale
        i++;
    }
    myfile.close();
    return res;

}


Eigen::Vector3d point3dToEigen(const cv::Point3d &point) {
    Eigen::Vector3d res = {point.x, point.y, point.z};
    return res;
}

Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matToEigen(const cv::Mat &matrix) {
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> res(matrix.rows, matrix.cols);

    for (u_int i_row = 0; i_row < matrix.rows; ++i_row) {
        for (u_int j_row = 0; j_row < matrix.cols; ++j_row) {
            res(i_row, j_row) = matrix.at<double>(i_row, j_row);
        }
    }
    return res;
}

cv::Point3d eigenToPoint3d(const Eigen::Vector3d &vector) {
    return cv::Point3d(vector[0], vector[1], vector[2]);
}

cv::Mat eigenToMat(const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> &matrix) {
    cv::Mat res = cv::Mat_<double>(matrix.rows(), matrix.cols());

    for (u_int i_row = 0; i_row < matrix.rows(); ++i_row) {
        for (u_int j_row = 0; j_row < matrix.cols(); ++j_row) {
            res.at<double>(i_row, j_row) = matrix(i_row, j_row);
        }
    }
    return res;
}

std::vector<std::vector<Eigen::Vector2d>> GetLast3TrackFeatures(const std::vector<cv::Point2f> &points_1_detect,
                                                                const std::vector<cv::Point2f> &points_2_matched,
                                                                const std::vector<cv::Point2f> &points_3_matched2,
                                                                const std::vector<uchar> &status_matched2) {
    const size_t points_num = points_1_detect.size();
    std::vector<std::vector<Eigen::Vector2d>> features_3_tracks;

    for (int i = 0; i< points_num; i++) {
        if (status_matched2[i] == 1) {
            std::vector<Eigen::Vector2d> tracks_3;

            Eigen::Vector2d p1(points_1_detect[i].x, points_1_detect[i].y);
            Eigen::Vector2d p2(points_2_matched[i].x, points_2_matched[i].y);
            Eigen::Vector2d p3(points_3_matched2[i].x, points_3_matched2[i].y);

            tracks_3.push_back(p1);
            tracks_3.push_back(p2);
            tracks_3.push_back(p3);
            features_3_tracks.push_back(tracks_3);
        }
    }
    if (features_3_tracks.size() < 10) {
        std::cout << "feature number no many";
    }
    assert(features_3_tracks.size() >= 3);

    return features_3_tracks;
}