#ifndef SCALE_H
#define SCALE_H

#include <Eigen/Dense>

double computeRelativeScaleFromLast3Tracks(
        std::vector<std::vector<Eigen::Vector2d>> &feature_track_last_3_positions,
        const Eigen::Matrix3d &c2_R_c1_prev,
        const Eigen::Vector3d &c2_p_c1_prev,
        const Eigen::Matrix3d &c2_R_c1_curr,
        const Eigen::Vector3d &c2_p_c1_curr);

double computeCurrRatio(const std::vector<Eigen::Vector2d> &last3_positions_feature1,
                        const std::vector<Eigen::Vector2d> &last3_positions_feature2,
                        const Eigen::Matrix3d &c2_R_c1_prev,
                        const Eigen::Vector3d &c2_p_c1_prev,
                        const Eigen::Matrix3d &c2_R_c1_curr,
                        const Eigen::Vector3d &c2_p_c1_curr);

Eigen::Vector3d triangulateWithRelativePoseUnitTranslation(const Eigen::Matrix3d &c2_R_c1,
                                                           const Eigen::Vector3d &c2_p_c1,
                                                           const Eigen::Vector2d &z1,
                                                           const Eigen::Vector2d &z2);

#endif