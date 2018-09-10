//
// Created by shaocheng on 9/7/18.
//

#ifndef HACKATHON_VO_FEATURES_H
#define HACKATHON_VO_FEATURES_H

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"


#include <iostream>
#include <ctype.h>
#include <algorithm> // for copy
#include <iterator> // for ostream_iterator
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>

class Features {
public:
    static void featureTracking(const cv::Mat &img_1,
                                const cv::Mat &img_2,
                                std::vector<cv::Point2f> *points1,
                                std::vector<cv::Point2f> *points2,
                                std::vector<uchar> *status);

    static void featureTrackingNoDeletion(const cv::Mat &img_1,
                                     const cv::Mat &img_2,
                                     std::vector<cv::Point2f> *points1,
                                     std::vector<cv::Point2f> *points2,
                                     std::vector<uchar> *status);

    static void featureDetection(const cv::Mat &img_1, std::vector<cv::Point2f> *points1);

};

#endif //HACKATHON_VO_FEATURES_H
