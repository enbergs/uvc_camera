#ifndef HACKATHON_VO_PROCESS_FRAME_H
#define HACKATHON_VO_PROCESS_FRAME_H

#include "features.h"
#include "utils.h"
#include "scale.h"
#include <queue>
#include "circular_frame_info_array.h"

class ProcessFrame {
private:
    // todo: parse from calib.txt
    const double kFocalLengthPX = 718.8560;
    const cv::Point2d kPrinciplePointPX = cv::Point2d(607.1928, 185.2157);
    const int kMinNumFeatures = 2000;
    const int kCircularArraySize = 5;

    cv::Mat E_mat_;
    cv::Mat R_;
    cv::Mat t_;
    cv::Mat R_prev_;
    cv::Mat t_prev_;
    cv::Mat mask_;
    double scale_abs_;

    CircularFrameInfoArray frame_info_vector_ = CircularFrameInfoArray(kCircularArraySize);

public:

    bool initialized_bool_ = false;
    bool ready_for_scale_ = false;

    void init(cv::Mat &img);

    void setScale(const double scale);

    void processImage(const cv::Mat &img_curr);

    double computeRelativeScale(const cv::Mat &img_curr, FrameInfo *frame_info_prev);

    FrameInfo getTopFrameInfo(int n);

};

#endif //HACKATHON_VO_PROCESS_FRAME_H
