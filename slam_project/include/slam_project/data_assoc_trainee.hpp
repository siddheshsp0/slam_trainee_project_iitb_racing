#ifndef DATA_ASSOC_TRAINEE_HPP
#define DATA_ASSOC_TRAINEE_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include <vector>
#include "dv_msgs/msg/cone.hpp"
#include "dv_msgs/msg/indexed_cone.hpp"
#include <numeric>
using Eigen::MatrixXd;
using Eigen::Matrix2d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::VectorXd;
using Eigen::VectorXi;

using namespace std;
struct JacobMatrices {
    MatrixXd zp, Hv, Hf, Sf;
};


std::vector<Eigen::Vector3d> known_landmarks = {
    // Blue cones (color code = 0)
    Eigen::Vector3d(-3.7219, -0.7307, 0),
    Eigen::Vector3d(0.2447, 2.1900, 0),
    Eigen::Vector3d(6.1094, 2.7757, 0),
    Eigen::Vector3d(22.6296, 5.5379, 0),
    Eigen::Vector3d(25.4920, 6.7404, 0),
    Eigen::Vector3d(28.1389, 6.2025, 0),
    Eigen::Vector3d(32.5034, 3.3361, 0),
    Eigen::Vector3d(33.8922, 1.4012, 0),
    Eigen::Vector3d(30.5221, 5.0812, 0),
    Eigen::Vector3d(34.4761, -1.5043, 0),
    Eigen::Vector3d(33.9992, -5.0234, 0),
    Eigen::Vector3d(32.9925, -8.0595, 0),
    Eigen::Vector3d(32.0983, -10.3000, 0),
    Eigen::Vector3d(30.1824, -13.5399, 0),
    Eigen::Vector3d(24.1140, -17.0441, 0),
    Eigen::Vector3d(8.9345, 3.0637, 0),
    Eigen::Vector3d(27.2831, -15.5644, 0),
    Eigen::Vector3d(21.2536, -18.8389, 0),
    Eigen::Vector3d(18.0618, -20.4551, 0),
    Eigen::Vector3d(14.4209, -22.2634, 0),
    Eigen::Vector3d(10.5025, -24.3305, 0),
    Eigen::Vector3d(7.2514, -26.4217, 0),
    Eigen::Vector3d(3.6516, -27.4551, 0),
    Eigen::Vector3d(0.7886, -26.9459, 0),
    Eigen::Vector3d(-1.4625, -25.2249, 0),
    Eigen::Vector3d(-3.2427, -23.5760, 0),
    Eigen::Vector3d(12.8688, 3.0125, 0),
    Eigen::Vector3d(-5.1431, -21.3860, 0),
    Eigen::Vector3d(-5.6174, -17.8608, 0),
    Eigen::Vector3d(-5.9382, -15.4551, 0),
    Eigen::Vector3d(-5.5558, -12.8602, 0),
    Eigen::Vector3d(-5.1206, -10.3000, 0),
    Eigen::Vector3d(-4.7841, -7.2575, 0),
    Eigen::Vector3d(-4.8432, -4.0291, 0),
    Eigen::Vector3d(-2.1864, 1.4137, 0),
    Eigen::Vector3d(16.5042, 3.4245, 0),
    Eigen::Vector3d(20.1368, 4.4270, 0),

    // Yellow cones (color code = 1)
    Eigen::Vector3d(0.7816, -2.6920, 1),
    Eigen::Vector3d(20.1479, -0.3734, 1),
    Eigen::Vector3d(23.4312, 0.4990, 1),
    Eigen::Vector3d(25.9655, 1.5014, 1),
    Eigen::Vector3d(27.9652, 0.9833, 1),
    Eigen::Vector3d(29.6054, -1.1797, 1),
    Eigen::Vector3d(6.0900, -1.8555, 1),
    Eigen::Vector3d(29.7063, -4.1923, 1),
    Eigen::Vector3d(28.8760, -6.8209, 1),
    Eigen::Vector3d(28.1060, -8.7973, 1),
    Eigen::Vector3d(26.6765, -10.3000, 1),
    Eigen::Vector3d(22.4195, -13.1474, 1),
    Eigen::Vector3d(25.0732, -11.7312, 1),
    Eigen::Vector3d(20.2628, -14.2408, 1),
    Eigen::Vector3d(17.6516, -15.4551, 1),
    Eigen::Vector3d(14.7877, -16.9672, 1),
    Eigen::Vector3d(11.0203, -18.8633, 1),
    Eigen::Vector3d(8.9831, -1.7968, 1),
    Eigen::Vector3d(7.8188, -20.8555, 1),
    Eigen::Vector3d(5.4296, -22.4125, 1),
    Eigen::Vector3d(3.2361, -23.0081, 1),
    Eigen::Vector3d(0.9662, -21.9718, 1),
    Eigen::Vector3d(-0.9298, -19.2829, 1),
    Eigen::Vector3d(-1.1575, -16.0733, 1),
    Eigen::Vector3d(-1.0043, -12.9300, 1),
    Eigen::Vector3d(-0.6087, -10.3000, 1),
    Eigen::Vector3d(-0.3478, -7.2429, 1),
    Eigen::Vector3d(12.9402, -1.9241, 1),
    Eigen::Vector3d(-0.3455, -4.5119, 1),
    Eigen::Vector3d(16.5151, -1.6032, 1),

    // Big Orange (color code = 2)
    Eigen::Vector3d(3.3869, 2.7000, 2),
    Eigen::Vector3d(3.0007, 2.6890, 2),
    Eigen::Vector3d(3.3785, -1.9068, 2),
    Eigen::Vector3d(3.0133, -1.9065, 2),
};



double normalize_angle(double angle);
JacobMatrices compute_jacobians(const std::vector<double>& mu_t, const Vector3d& xf, const Matrix2d& Pf, const Eigen::Matrix2d& Q_cov);
bool performICTest(const std::vector<double> &mu_t, int lm_id, const Eigen::Vector2d &z, const Eigen::Matrix2d &Q_cov);
double computeObservationLikelihood(const std::vector<double> &mu_t, int lm_id, const Eigen::Vector2d &z, const Eigen::Matrix2d &Q_cov);
std::vector<int> performDataAssociation(
    const std::vector<double>& mu_t,
    const std::vector<dv_msgs::msg::IndexedCone>& conesFromPerception,
    const Eigen::Matrix2d& Q_cov);


#endif