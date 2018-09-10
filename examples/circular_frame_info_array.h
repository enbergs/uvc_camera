#ifndef HACKATHON_VO_CIRCULAR_FRAME_INFO_ARRAY_H
#define HACKATHON_VO_CIRCULAR_FRAME_INFO_ARRAY_H


#include <Eigen/Dense>
#include "features.h"


struct FrameInfo {
    int index;
    cv::Mat img;
    cv::Mat g_R;
    cv::Mat g_t;


    std::vector<cv::Point2f> features_detected;
    std::vector<cv::Point2f> features_matched;
    std::vector<cv::Point2f> features_matched_2;

    FrameInfo() {}

    FrameInfo(int index,
              const cv::Mat &img,
              const cv::Mat &g_R,
              const cv::Mat &g_t,
              const std::vector<cv::Point2f> &features_detected,
              const std::vector<cv::Point2f> &features_matched,
              const std::vector<cv::Point2f> &features_matched_2) {

        this->index = index;
        this->img = img;
        this->g_R = g_R;
        this->g_t = g_t;
        this->features_detected = features_detected;
        this->features_matched = features_matched;
        this->features_matched_2 = features_matched_2;
    }
};

class CircularFrameInfoArray {

private:
    int max_size_;
    std::vector<FrameInfo> circular_array_;
    int curr_idx_ = 0;

public:
    CircularFrameInfoArray(int max_size);

    void push(const FrameInfo &frame_info);

    FrameInfo& getNSteps(const int n);

    FrameInfo& getPrev();

};


#endif //HACKATHON_VO_CIRCULAR_FRAME_INFO_ARRAY_H
