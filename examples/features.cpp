#include "features.h"

void Features::featureTracking(const cv::Mat &img_1,
                     const cv::Mat &img_2,
                     std::vector<cv::Point2f> *points1,
                     std::vector<cv::Point2f> *points2,
                     std::vector<uchar> *status) {

//this function automatically gets rid of points for which tracking fails

    std::vector<float> err;
    cv::Size winSize = cv::Size(21, 21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    calcOpticalFlowPyrLK(img_1, img_2, *points1, *points2, *status, err, winSize, 3, term_criteria, 0, 0.001);
//    std::cout << "features_prev.size() = " << points1->size() << std::endl;
//    std::cout << "features_curr.size() = " << points2->size() << std::endl;
    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    int index_correction = 0;
    for (int i = 0; i < status->size(); i++) {
        cv::Point2f pt = points2->at(i - index_correction);
        if ((status->at(i) == 0) || (pt.x < 0) || (pt.y < 0)) {
            if ((pt.x < 0) || (pt.y < 0)) {
                status->at(i) = 0;
            }
            points1->erase(points1->begin() + (i - index_correction));
            points2->erase(points2->begin() + (i - index_correction));
            index_correction++;
        }

    }

}


void Features::featureTrackingNoDeletion(const cv::Mat &img_1,
                                       const cv::Mat &img_2,
                                       std::vector<cv::Point2f> *points1,
                                       std::vector<cv::Point2f> *points2,
                                       std::vector<uchar> *status) {

//this function automatically gets rid of points for which tracking fails

    std::vector<float> err;
    cv::Size winSize = cv::Size(21, 21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);

    calcOpticalFlowPyrLK(img_1, img_2, *points1, *points2, *status, err, winSize, 3, term_criteria, 0, 0.001);

    //getting rid of points for which the KLT tracking failed or those who have gone outside the frame
    for (int i = 0; i < status->size(); i++) {
        cv::Point2f pt = points2->at(i);
        if (status->at(i) != 0){
            if ((pt.x < 0) || (pt.y < 0)) {
                status->at(i) = 0;
            }
        }
    }

}



void Features::featureDetection(const cv::Mat &img_1, std::vector<cv::Point2f> *points1) {   //uses FAST as of now, modify parameters as necessary
    std::vector<cv::KeyPoint> keypoints_1;
    int fast_threshold = 50;
    bool nonmax_suppression = true;
    FAST(img_1, keypoints_1, fast_threshold, nonmax_suppression);
    cv::KeyPoint::convert(keypoints_1, *points1, std::vector<int>());
}

