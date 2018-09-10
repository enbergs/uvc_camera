#ifndef HACKATHON_VO_DEMO_H
#define HACKATHON_VO_DEMO_H

#include "features.h"

class Demo {

    const int font_face = cv::FONT_HERSHEY_PLAIN;
    const double font_scale = 1.5;
    const int thickness = 1;
    const std::string image_stream_str = "Image Stream";
    const std::string trajectory_str = "Trajectory";
    const cv::Point text_origin = cv::Point(10, 50);

    int image_width = 1200;
    int image_height = 1400;

    char text[200];

    cv::Mat traj = cv::Mat::zeros(image_width, image_height, CV_8UC3);

public:

    void initWindows();

    void showTraj(const cv::Mat &coord_curr, const cv::Mat &coord_prev);

    void showImage(const cv::Mat &img);

    void computeCoordInImage(const cv::Mat &t_f, int *x, int *y);

};

#endif //HACKATHON_VO_DEMO_H
