#ifndef RP_DEVELOPABLIZE_H
#define RP_DEVELOPABLIZE_H

#include <Eigen/Core>
#include <vector>

// zeroFormVは特異点なしの必要あり。angleではなくreal numberで。
namespace rp {
  void developablize(const Eigen::MatrixX3i &F, const Eigen::MatrixX3d &V,
                     const std::vector<std::vector<size_t>> &paths,
                     // ある面が特異点かどうか
                     /*const Eigen::VectorXi &isSingularity, const Eigen::VectorXd &zeroFormV,*/
                     Eigen::MatrixX3i &resultF, Eigen::MatrixX3d &resultV);
}

#endif //RP_DEVELOPABLIZE_H