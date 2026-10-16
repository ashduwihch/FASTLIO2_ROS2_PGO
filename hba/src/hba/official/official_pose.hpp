#pragma once
#include <Eigen/Dense>
namespace official_hba_types {
struct Pose {
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  Eigen::Quaterniond q{1.0, 0.0, 0.0, 0.0};
  Eigen::Vector3d t{Eigen::Vector3d::Zero()};
};
}
