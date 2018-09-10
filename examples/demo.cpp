#include "demo.h"

void Demo::initWindows() {
    cv::namedWindow(image_stream_str, cv::WINDOW_AUTOSIZE); // Create a window for display.
    cv::namedWindow(trajectory_str, cv::WINDOW_AUTOSIZE); // Create a window for display.
}

void Demo::showTraj(const cv::Mat &coord_curr, const cv::Mat &coord_prev) {
    int x_curr, y_curr;
    int x_prev, y_prev;

    computeCoordInImage(coord_curr, &x_curr, &y_curr);
    computeCoordInImage(coord_prev, &x_prev, &y_prev);

    circle(traj, cv::Point(x_prev, y_prev), 2, CV_RGB(0, 0, 0), 4);
    circle(traj, cv::Point(x_prev, y_prev), 2, CV_RGB(255, 0, 0), 4);

    circle(traj, cv::Point(x_curr, y_curr), 2, CV_RGB(255, 255, 255), 4);


    // draw a rectangle that covers previous text
    rectangle(traj, cv::Point(10, 30), cv::Point(700, 65), CV_RGB(0, 0, 0), CV_FILLED);
    sprintf(text, "x = %02.3f, y = %02.3f, z = %02.3f", coord_curr.at<double>(0), coord_curr.at<double>(1),
            coord_curr.at<double>(2));
    putText(traj, text, text_origin, font_face, font_scale, cv::Scalar::all(255), thickness, 12);

    imshow(trajectory_str, traj);
}

void Demo::showImage(const cv::Mat &img) {
    imshow(image_stream_str, img);
}

void Demo::computeCoordInImage(const cv::Mat &t_f, int *x, int *y) {
    *x = int(t_f.at<double>(0)) + image_width / 2;
    *y = int(t_f.at<double>(2)) + image_height / 2;
}