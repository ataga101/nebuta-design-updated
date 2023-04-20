//
// Created by Naoki Agata on 2023/04/07.
//

#include "flatten_triangle_strip.h"
#include "Eigen/Core"
#include "igl/per_face_normals.h"
#include "igl/triangle_triangle_adjacency.h"
#include "math.h"
#include "assert.h"

void assert_length_preservation(
        Eigen::Vector3d v1,
        Eigen::Vector3d v2,
        Eigen::Vector3d v3,
        Eigen::Vector3d v1_orig,
        Eigen::Vector3d v2_orig,
        Eigen::Vector3d v3_orig
){
    return;
    assert(std::abs((v1 - v2).norm() - (v1_orig - v2_orig).norm()) <= 10e-4);
    assert(std::abs((v2 - v3).norm() - (v2_orig - v3_orig).norm()) <= 10e-4);
    assert(std::abs((v3 - v1).norm() - (v3_orig - v1_orig).norm()) <= 10e-4);
}

void fit_triangle_in_2d(Eigen::MatrixXd &V, \
                    Eigen::MatrixXi &F, \
                    Eigen::MatrixXd &N, \
                    Eigen::MatrixXi &TT, \
                    Eigen::MatrixXi &TTi, \
                    Eigen::Index Fid, \
                    Eigen::Index v0_F, \
                    Eigen::Index v1_F, \
                    Eigen::Index v2_F, \
                    Eigen::Index former_v, \
                    Eigen::MatrixXd &V_UV, \
                    Eigen::VectorXi &visitedF, \
                    Eigen::VectorXi &visitedV){
    if(visitedF(Fid) == 1){
        return;
    }
    visitedF(Fid) = 1;

    Eigen::Index v0 = F(Fid, v0_F);
    Eigen::Index v1 = F(Fid, v1_F);
    Eigen::Index v2 = F(Fid, v2_F);

    assert(visitedV(v0) == 1);
    assert(visitedV(v1) == 1);
    assert(visitedV(v2) == 0);
    assert(visitedV(former_v) == 1);
    assert(v0 != former_v && v1 != former_v && v2 != former_v);
    visitedV(v2) = 1;

    Eigen::Vector3d v0v1_vec_3d = V.row(v1) - V.row(v0);
    Eigen::Vector3d v0v2_vec_3d = V.row(v2) - V.row(v0);
    double v2_projection_onto_v0v1_length = v0v2_vec_3d.dot(v0v1_vec_3d) / v0v1_vec_3d.norm();
    double v2_normal_to_v0v1_length = std::sqrt(v0v2_vec_3d.norm() * v0v2_vec_3d.norm() - v2_projection_onto_v0v1_length * v2_projection_onto_v0v1_length);

    Eigen::Vector2d v0v1_vec_2d = V_UV.row(v1) - V_UV.row(v0);
    Eigen::Vector2d v0v1_orthogonal_vec_2d = Eigen::Vector2d(-v0v1_vec_2d(1), v0v1_vec_2d(0));
    Eigen::Vector2d v0_pos_2d = V_UV.row(v0);

    Eigen::Vector2d candidate_uv_projection_along_v0v1 = v0_pos_2d + v0v1_vec_2d.normalized() * v2_projection_onto_v0v1_length;
    Eigen::Vector2d candidate_uv_normal_to_v0v1 = v0v1_orthogonal_vec_2d.normalized() * v2_normal_to_v0v1_length;
    Eigen::Vector2d former_v_uv = V_UV.row(former_v);
    Eigen::Vector2d candidate1 = candidate_uv_projection_along_v0v1 + candidate_uv_normal_to_v0v1;
    Eigen::Vector2d candidate2 = candidate_uv_projection_along_v0v1 - candidate_uv_normal_to_v0v1;

    if((candidate1 - former_v_uv).norm() < (candidate2 - former_v_uv).norm()) {
        V_UV(v2, 0) = candidate2(0);
        V_UV(v2, 1) = candidate2(1);
    } else {
        V_UV(v2, 0) = candidate1(0);
        V_UV(v2, 1) = candidate1(1);
    }

    assert_length_preservation(Eigen::Vector3d(V_UV(v0, 0), V_UV(v0, 1), 0), \
                               Eigen::Vector3d(V_UV(v1, 0), V_UV(v1, 1), 0), \
                               Eigen::Vector3d(V_UV(v2, 0), V_UV(v2, 1), 0), \
                               Eigen::Vector3d(V(v0, 0), V(v0, 1), V(v0, 2)), \
                               Eigen::Vector3d(V(v1, 0), V(v1, 1), V(v1, 2)), \
                               Eigen::Vector3d(V(v2, 0), V(v2, 1), V(v2, 2)));

    // Recursively apply this
    for(int i=0; i<3; i++){
        Eigen::Index new_Fid = TT(Fid, i);

        if(new_Fid != -1){
            Eigen::Index new_v0_F = TTi(Fid, i);
            Eigen::Index new_v1_F = (new_v0_F + 1) % 3;
            Eigen::Index new_v2_F = (new_v0_F + 2) % 3;
            fit_triangle_in_2d(V, F, N, TT, TTi, new_Fid, new_v0_F, new_v1_F, new_v2_F, F(Fid, (i+2)%3), V_UV, visitedF, visitedV);
        }
    }
}

void flatten_triangle_strip(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::VectorXd orig_normals, Eigen::MatrixXd &V_UV){
    Eigen::MatrixXd N;
    igl::per_face_normals(V, F, N);

    V_UV.resize(V.rows(), 2);

    if(N.colwise().mean().dot(orig_normals) < 0){
        N = -N;
    }

    Eigen::MatrixXi TT, TTi;
    igl::triangle_triangle_adjacency(F, TT, TTi);

    V_UV = Eigen::MatrixXd::Zero(V.rows(), 2);
    Eigen::VectorXi visitedF = Eigen::VectorXi::Zero(F.rows());
    Eigen::VectorXi visitedV = Eigen::VectorXi::Zero(V.rows());

    // Flatten the first triangle
    Eigen::Index Fid = 0;
    visitedF(Fid) = 1;
    for(int i=0; i<3; i++){
        Eigen::Index v = F(Fid, i);
        visitedV(v) = 1;
    }

    Eigen::Index v0 = F(Fid, 0);
    Eigen::Index v1 = F(Fid, 1);
    Eigen::Index v2 = F(Fid, 2);

    Eigen::Vector3d now_normal = N.row(Fid);
    Eigen::Vector3d target_normal = Eigen::Vector3d::UnitZ();

    Eigen::Vector3d axis = now_normal.cross(target_normal).normalized();
    double angle = std::acos(now_normal.dot(target_normal));
    Eigen::Matrix3d rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

    Eigen::Vector3d v0_pos = V.row(v0);
    Eigen::Vector3d v1_pos = V.row(v1);
    Eigen::Vector3d v2_pos = V.row(v2);

    // Rotate the triangle to the XY plane
    Eigen::Vector3d v0_pos_rot = (axis.norm() != 0) ? rot * v0_pos : v0_pos;
    Eigen::Vector3d v1_pos_rot = (axis.norm() != 0) ? rot * v1_pos : v1_pos;
    Eigen::Vector3d v2_pos_rot = (axis.norm() != 0) ? rot * v2_pos : v2_pos;

    V_UV(v0, 0) = v0_pos_rot(0);
    V_UV(v0, 1) = v0_pos_rot(1);
    V_UV(v1, 0) = v1_pos_rot(0);
    V_UV(v1, 1) = v1_pos_rot(1);
    V_UV(v2, 0) = v2_pos_rot(0);
    V_UV(v2, 1) = v2_pos_rot(1);

    // Recursively apply this
    for(int i=0; i<3; i++){
        Eigen::Index new_Fid = TT(Fid, i);

        if(new_Fid != -1){
            Eigen::Index new_v0_F = TTi(Fid, i);
            Eigen::Index new_v1_F = (new_v0_F + 1) % 3;
            Eigen::Index new_v2_F = (new_v0_F + 2) % 3;
            fit_triangle_in_2d(V, F, N, TT, TTi, new_Fid, new_v0_F, new_v1_F, new_v2_F, F(Fid, (i+2)%3), V_UV, visitedF, visitedV);
        }
    }

    for(int i=0;i<F.rows();i++){
        assert(visitedF(i) == 1);
    }
}

