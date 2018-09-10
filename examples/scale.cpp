#include "scale.h"

/* compute the median of a list of scale candidates */
double computeRelativeScaleFromLast3Tracks(
        std::vector<std::vector<Eigen::Vector2d>> &feature_track_last_3_positions,
        const Eigen::Matrix3d &c2_R_c1_prev,
        const Eigen::Vector3d &unit_c2_p_c1_prev,
        const Eigen::Matrix3d &c2_R_c1_curr,
        const Eigen::Vector3d &unit_c2_p_c1_curr) {

    const size_t track_size = feature_track_last_3_positions.size();
    // make sure # of feature tracks is not very few
    assert(track_size >= 3);

    // collection of ratios to find the median as relative scale
    std::vector<double> all_ratio;

    for (size_t i = 0; i < track_size - 1; i++) {
        double curr_ratio = computeCurrRatio(feature_track_last_3_positions[i],
                                             feature_track_last_3_positions[i + 1],
                                             c2_R_c1_prev,
                                             unit_c2_p_c1_prev,
                                             c2_R_c1_curr,
                                             unit_c2_p_c1_curr);
        if (curr_ratio < 0.01 || curr_ratio > 100) {
            continue;
        }
        all_ratio.push_back(curr_ratio);
    }

    // todo: have the median more efficiently
    std::sort(all_ratio.begin(), all_ratio.end());

    // totally have "track_size -1" ratios to work on
    size_t m = (all_ratio.size() - 1) / 2;

    return ((all_ratio.size() - 1) % 2 == 0) ? (all_ratio[m] + all_ratio[m - 1]) / 2.0 : all_ratio[m];
}


double computeCurrRatio(const std::vector<Eigen::Vector2d> &last3_positions_feature1,
                        const std::vector<Eigen::Vector2d> &last3_positions_feature2,
                        const Eigen::Matrix3d &c2_R_c1_prev,
                        const Eigen::Vector3d &unit_c2_p_c1_prev,
                        const Eigen::Matrix3d &c2_R_c1_curr,
                        const Eigen::Vector3d &unit_c2_p_c1_curr) {

    //TODO: make sure the feature coordinates are undistorted

    Eigen::Vector3d c1_p_f1 = triangulateWithRelativePoseUnitTranslation(c2_R_c1_prev,
                                                                         unit_c2_p_c1_prev,
                                                                         last3_positions_feature1[0],
                                                                         last3_positions_feature1[1]);

    Eigen::Vector3d c1_p_f2 = triangulateWithRelativePoseUnitTranslation(c2_R_c1_prev,
                                                                         unit_c2_p_c1_prev,
                                                                         last3_positions_feature2[0],
                                                                         last3_positions_feature2[1]);

    Eigen::Vector3d c2_p_f1 = triangulateWithRelativePoseUnitTranslation(c2_R_c1_curr,
                                                                         unit_c2_p_c1_curr,
                                                                         last3_positions_feature1[1],
                                                                         last3_positions_feature1[2]);

    Eigen::Vector3d c2_p_f2 = triangulateWithRelativePoseUnitTranslation(c2_R_c1_curr,
                                                                         unit_c2_p_c1_curr,
                                                                         last3_positions_feature2[1],
                                                                         last3_positions_feature2[2]);
    // compute the relative_scale according to the distance (norm) ratio
    double norm1 = (c1_p_f2 - c1_p_f1).norm();
    double norm2 = (c2_p_f2 - c2_p_f1).norm();
    assert(norm1 > 0.0001);

    return norm2 / norm1;
}


Eigen::Vector3d triangulateWithRelativePoseUnitTranslation(const Eigen::Matrix3d &c2_R_c1,
                                                           const Eigen::Vector3d &unit_c2_p_c1,
                                                           const Eigen::Vector2d &z1,
                                                           const Eigen::Vector2d &z2) {

    Eigen::Matrix<double, 2, 3> Intrinsics_truc;
    Intrinsics_truc << 718.8560, 0, 607.1928, 0, 718.8560, 185.2157;
    // z1, z2 are two meas in consecutive frames for same feature
    assert(abs(unit_c2_p_c1.norm() - 1.0) < 1e-6);
    Eigen::Matrix<double, 2, 3> A1;
    A1 = Intrinsics_truc;
    A1.col(2) -= z1;

    Eigen::Matrix<double, 2, 3> A2_tmp;
    A2_tmp = Intrinsics_truc;
    A2_tmp.col(2) -= z2;
    Eigen::MatrixXd A2 = A2_tmp * c2_R_c1;

    Eigen::Vector4d b = Eigen::Vector4d::Zero();
    b.segment<2>(2) = -A2_tmp * unit_c2_p_c1;

    Eigen::Matrix<double, 4, 3> A;
    A << A1, A2;

    Eigen::Vector3d c1_p_f = A.colPivHouseholderQr().solve(b);
    //TODO add GN iterations to refine the estimate
    return c1_p_f;
}