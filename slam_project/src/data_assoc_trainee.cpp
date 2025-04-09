#include "slam_project/data_assoc_trainee.hpp"

double normalize_angle(double angle) {
    const double PI = std::acos(-1);  
    angle =
        std::fmod(angle, 2 * PI);  
  
    // Shift angle to be within the range [-PI, PI)
    if (angle >= PI) {
      angle -= 2 * PI;
    } else if (angle < -PI) {
      angle += 2 * PI;
    }
  
    return angle;
}

JacobMatrices compute_jacobians(const std::vector<double>& mu_t, const Vector3d& xf, const Matrix2d& Pf, const Eigen::Matrix2d& Q_cov)
 {

    JacobMatrices ans;

    // Compute the relative position between the robot and the landmark
    double dx = xf(0) - mu_t[0];
    double dy = xf(1) - mu_t[1];
    double d2 = dx * dx + dy * dy;
    double d = std::sqrt(d2); // Euclidean distance (range)

    // Predicted measurement: range (d) and bearing (phi)
    ans.zp = MatrixXd(2, 1);
    ans.zp << d, normalize_angle(atan2(dy, dx) - mu_t[2]);

    // Jacobian of the measurement with respect to the robot state
    ans.Hv = MatrixXd(2, 3);
    ans.Hv << 
        -dx / d, -dy / d, 0.0,
        dy / d2, -dx / d2, -1.0;

    // Jacobian of the measurement with respect to the landmark state
    ans.Hf = MatrixXd(2, 2);
    ans.Hf << 
        dx / d, dy / d,
        -dy / d2, dx / d2;

    // Measurement covariance
    ans.Sf = ans.Hf * Pf * ans.Hf.transpose() + Q_cov;

    return ans;
}

bool performICTest(const std::vector<double> &mu_t, int lm_id, const Eigen::Vector2d &z, const Eigen::Matrix2d &Q_cov) {

    Eigen::Vector3d xf = known_landmarks[lm_id]; // Landmark position
    Eigen::Matrix2d Pf;
    Pf << 0.001, 0.0, 0.0, 0.001; // Landmark covariance

    JacobMatrices jm = compute_jacobians(mu_t, xf, Pf, Q_cov);

    Eigen::Vector2d dz;
    dz << z(0) - jm.zp(0), normalize_angle(z(1) - jm.zp(1));

    Eigen::Matrix2d Sf_inv = jm.Sf.inverse();
    double mahal_dist = dz.transpose() * Sf_inv * dz;

    return (mahal_dist <= 6.8);
}

double computeObservationLikelihood(const std::vector<double> &mu_t, int lm_id, const Eigen::Vector2d &z, const Eigen::Matrix2d &Q_cov) {

    Eigen::Vector3d xf=known_landmarks[lm_id]; // Landmark position
    Eigen::Matrix2d Pf;
    Pf <<0.001,0.0,0.0,0.001; // Landmark covariance

    JacobMatrices jm = compute_jacobians(mu_t, xf, Pf, Q_cov);

    Eigen::Vector2d dz;
    dz << z(0) - jm.zp(0), normalize_angle(z(1) - jm.zp(1));

    double den = 2.0 * M_PI * std::sqrt(jm.Sf.determinant());
    if (den <= 0.0 || std::isnan(den)) {
        // RCLCPP_INFO_STREAM(this->get_logger(), "Invalid denominator. Returning small value.");
        return 1e-12;
    }

    double exponent = -0.5 * dz.transpose() * jm.Sf.inverse() * dz;
    double likelihood = std::exp(exponent) / den;

    return likelihood;
}

std::vector<int> performDataAssociation(
    const std::vector<double>& mu_t,
    const std::vector<dv_msgs::msg::IndexedCone>& conesFromPerception,
    const Eigen::Matrix2d& Q_cov){ 

    std::vector<int> matchedConeIndices;

    std::vector<size_t> obs_indices(conesFromPerception.size());
    std::iota(obs_indices.begin(), obs_indices.end(), 0);
    // std::sort(obs_indices.begin(), obs_indices.end(), [&](size_t a, size_t b) {
    //     return conesFromPerception[a].range < conesFromPerception[b].range;
    // });
    for (size_t obs_idx : obs_indices) {
        const auto &obs = conesFromPerception[obs_idx];
        Eigen::Vector2d z(obs.location.x, obs.location.y);

        int best_landmark = -1;
        double best_likelihood = 0.0;

        // Filter landmarks using IC test and compute likelihood
        for (int lm_id = 0; lm_id < known_landmarks.size(); ++lm_id) {

            if (obs.color != known_landmarks[lm_id](2)) {
                continue;
            }

            if (performICTest(mu_t, lm_id, z, Q_cov)) {
                // IC test passed, now compute likelihood
                double likelihood = computeObservationLikelihood(mu_t, lm_id, z, Q_cov);

                if (likelihood > best_likelihood) {
                    best_likelihood = likelihood;
                    best_landmark = lm_id;
                }
            }   
        }
        // If no suitable landmark found or likelihood is below the threshold, mark as new landmark
        if (best_landmark == -1 || best_likelihood < 0.005) matchedConeIndices.push_back(-1); // New landmark
        else matchedConeIndices.push_back(best_landmark);
    }
    // std::ostringstream oss;
    // oss << "Matched Cone Indices: ";
    // for (const auto& index : matchedConeIndices) {
    //     oss << index << " ";
    // }

    return matchedConeIndices;
}