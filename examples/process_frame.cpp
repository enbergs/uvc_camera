#include "process_frame.h"


void ProcessFrame::init(const cv::Mat &img) {

    // first image, only detect features_detected, no feature tracking process
    std::vector<cv::Point2f> feature_points_detected;
    Features::featureDetection(img, &feature_points_detected);
    // R_ = I and t_ = 0
    cv::Mat identity_mat = cv::Mat::eye(3, 3, CV_64FC1);
    cv::Mat zero_translation = cv::Mat::zeros(3, 1, CV_64FC1);
    // zip into FrameInfo
    FrameInfo first_frame_info = FrameInfo(0, img, identity_mat.clone(), zero_translation.clone(), feature_points_detected, {}, {});
    frame_info_vector_.push(first_frame_info);
    // set initialization flag to be true
    initialized_bool_ = true;
}

void ProcessFrame::setScale(const double scale) {
    scale_abs_ = scale;
    std::cout << "Abs scale is " << scale << std::endl;
}


void ProcessFrame::processImage(const cv::Mat &img_curr) {
    // sanity check if it is initialized
    assert(initialized_bool_);

    // get previous
    FrameInfo &frame_info_prev = frame_info_vector_.getPrev();

    std::vector<uchar> status;

    std::vector<cv::Point2f> feature_points_detected_curr;
    Features::featureDetection(img_curr, &feature_points_detected_curr);

    printf("%zu features detected\n", feature_points_detected_curr.size());

    std::vector<cv::Point2f> feature_points_matched_curr;

    unsigned int min_features = 20;
    bool stand_chance = (frame_info_prev.features_detected.size() > min_features);

    if (stand_chance) {
        Features::featureTracking(frame_info_prev.img, img_curr, &frame_info_prev.features_detected, &feature_points_matched_curr, &status);
        E_mat_ = findEssentialMat(frame_info_prev.features_detected,
            feature_points_matched_curr,
            kFocalLengthPX,
            kPrinciplePointPX,
            cv::RANSAC, 0.999, 1.0, mask_);
        recoverPose(E_mat_, frame_info_prev.features_detected, feature_points_matched_curr, R_, t_, kFocalLengthPX, kPrinciplePointPX,
            mask_);
    } else {
        R_ = cv::Mat::eye(3, 3, CV_64FC1);
        t_ = cv::Mat::zeros(3, 1, CV_64FC1);
        printf("we did not stand a chance again ******************************************************\n");
    }

    cv::Mat t_curr = frame_info_prev.g_t;
    cv::Mat R_curr = frame_info_prev.g_R;

    if (!ready_for_scale_) {
        ready_for_scale_ = true;
        scale_abs_ = 1;
        t_curr = scale_abs_ * (frame_info_prev.g_R * t_) + frame_info_prev.g_t;
        R_curr = frame_info_prev.g_R * R_;
        FrameInfo frame_info_curr(frame_info_prev.index + 1,
                                  img_curr.clone(),
                                  R_curr.clone(),
                                  t_curr.clone(),
                                  feature_points_detected_curr,
                                  feature_points_matched_curr, {});
        frame_info_vector_.push(frame_info_curr);
        R_prev_ = R_.clone();
        t_prev_ = t_.clone();
        return;
    }
    // after third frame


    FrameInfo frame_info_prev_prev = frame_info_vector_.getNSteps(2);

    // match matched features in img_2 for img_3
    std::vector<uchar> status_matched2check;
    std::vector<cv::Point2f> features_matched_2_curr;
    // compute relative scale

    Eigen::Matrix3d c2_R_c1_prev = matToEigen(R_prev_).transpose();
    Eigen::Vector3d unit_c2_p_c1_prev = -c2_R_c1_prev * matToEigen(t_prev_);
    Eigen::Matrix3d c2_R_c1_curr = matToEigen(R_).transpose();
    Eigen::Vector3d unit_c2_p_c1_curr = -c2_R_c1_curr * matToEigen(t_);

    stand_chance = (frame_info_prev.features_matched.size() > min_features);
    if (stand_chance) {
        Features::featureTrackingNoDeletion(frame_info_prev.img,
            img_curr,
            &frame_info_prev.features_matched,
            &features_matched_2_curr,
            &status_matched2check); //track those features to img_2

        std::vector<std::vector<Eigen::Vector2d>> features_last_3_tracks = GetLast3TrackFeatures(frame_info_prev_prev.features_detected,
            frame_info_prev.features_matched,
            features_matched_2_curr,
            status_matched2check);

        double norm1 = unit_c2_p_c1_prev.norm();
        double norm2 = unit_c2_p_c1_curr.norm();
        double eps = 1.0e-5;
        bool are_we_ok = (fabs(norm1 - 1.0) < eps) && (fabs(norm2 - 1.0) < eps) && (features_last_3_tracks.size() >= 3);
        if (are_we_ok) {
            double scale_rel = computeRelativeScaleFromLast3Tracks(features_last_3_tracks, c2_R_c1_prev, unit_c2_p_c1_prev, c2_R_c1_curr, unit_c2_p_c1_curr);
            scale_abs_ *= scale_rel;
            std::cout << "Computed: scale_rel = " << scale_rel << ",scale_abs = " << scale_abs_<< std::endl;
        } // done computing relative scale
    }

    // force the dynamics to be correct
    double default_scale = 1.0241;
    if (scale_abs_ < 0.2 || scale_abs_ > 2) {
        scale_abs_ = default_scale;
    }

    bool mostly_in_z = std::abs(t_.at<double>(2)) > t_.at<double>(0) && std::abs(t_.at<double>(2)) > t_.at<double>(1);
    if ((scale_abs_ > 0.1) && mostly_in_z) {
        t_curr = scale_abs_ * (R_curr * t_) + t_curr;
        R_curr = R_curr * R_;
    } else {
        // t_curr = cv::Mat::zeros(3, 1, CV_64FC1);
        // R_curr = cv::Mat::eye(3, 3, CV_64FC1);
        std::cout << "scale below 0.1, or incorrect translation" << std::endl;
    }

    // lines for printing results
    std::cout << t_curr.at<double>(0) << " " << t_curr.at<double>(1) << " " << t_curr.at<double>(2) << std::endl;

    FrameInfo frame_info_curr(frame_info_prev.index + 1,
                              img_curr.clone(),
                              R_curr.clone(),
                              t_curr.clone(),
                              feature_points_detected_curr,
                              feature_points_matched_curr,
                              features_matched_2_curr);
    frame_info_vector_.push(frame_info_curr);
    R_prev_ = R_.clone();
    t_prev_ = t_.clone();
}

FrameInfo ProcessFrame::getTopFrameInfo(int i) {
    return frame_info_vector_.getNSteps(i);
}