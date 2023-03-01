#pragma once

#include <Eigen/Core>
#include <vector>

// zeroFormVは特異点なしの必要あり。angleではなくreal numberで。
namespace rp_quadric {
  void developablize(const Eigen::MatrixX3i &F, const Eigen::MatrixX3d &V,
                     const std::vector<std::vector<int>> &paths,
                     // ある面が特異点かどうか
                     /*const Eigen::VectorXi &isSingularity, const Eigen::VectorXd &zeroFormV,*/
                     Eigen::MatrixX3i &resultF, Eigen::MatrixX3d &resultV);

  void approximate_single_patch(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                                Eigen::MatrixXd &resultV, Eigen::MatrixXi &resultF);
}
