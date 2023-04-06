//
// Created by Naoki Agata on 2023/03/02.
//

#ifndef NEBUTA_DESIGNER_APPROXIMATE_SINGLE_PATCH_H
#define NEBUTA_DESIGNER_APPROXIMATE_SINGLE_PATCH_H

#include "triangle_strips.h"
#include "developablize.h"
#include "developablize_quadric.h"

namespace approximate_single_patch {

    enum approximation_mode {
        MITANI, QSlim, DP_normal, DP_perface_distance
    };
    approximation_mode approx_mode;

    void approximate_single_patch(Eigen::MatrixXd &V, Eigen::MatrixXi &F,
                                  Eigen::MatrixXd &resultV, Eigen::MatrixXi &resultF) {
        if (approx_mode == MITANI) {
            rp::approximate_single_patch(V, F, resultV, resultF);
        } else if (approx_mode == QSlim) {
            rp_quadric::approximate_single_patch(V, F, resultV, resultF);
        } else if (approx_mode == DP_normal) {
            tri_strip::approximate_single_patch(V, F, resultV, resultF, tri_strip::normal_alignment);
        } else if (approx_mode == DP_perface_distance) {
            tri_strip::approximate_single_patch(V, F, resultV, resultF, tri_strip::per_face_distance);
        }
    }
}

#endif //NEBUTA_DESIGNER_APPROXIMATE_SINGLE_PATCH_H
