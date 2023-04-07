//
// Created by Naoki Agata on 2023/04/05.
//

#ifndef NEBUTA_DESIGNER_FLATTEN_TRIANGLE_STRIP_H
#define NEBUTA_DESIGNER_FLATTEN_TRIANGLE_STRIP_H

#include "Eigen/Core"

void flatten_triangle_strip(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::VectorXd orig_normals, Eigen::MatrixXd &V_UV);

#endif //NEBUTA_DESIGNER_FLATTEN_TRIANGLE_STRIP_H
