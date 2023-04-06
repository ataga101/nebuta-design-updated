//
// Created by Naoki Agata on 2023/04/05.
//

#ifndef NEBUTA_DESIGNER_FLATTEN_TRIANGLE_STRIP_H
#define NEBUTA_DESIGNER_FLATTEN_TRIANGLE_STRIP_H

#include "Eigen/Core"
#include "igl/per_face_normals.h"
#include "igl/triangle_triangle_adjacency.h"
#include "math.h"
#include "assert.h"

void fit_triangle_in_2d(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &N, Eigen::MatrixXi &TT, Eigen::MatrixXi &TTi, Eigen::Index Fid, Eigen::Index v0_F, Eigen::Index v1_F, Eigen::Index v2_F, Eigen::MatrixXd &V_UV, Eigen::VectorXi &visited){
    if(visited(Fid) == 1){
        return;
    }
    visited(Fid) = 1;

    Eigen::Index v0 = F(Fid, v0_F);
    Eigen::Index v1 = F(Fid, v1_F);
    Eigen::Index v2 = F(Fid, v2_F);

    Eigen::Vector3d now_normal = N.row(Fid);
    Eigen::Vector3d target_normal = Eigen::Vector3d::UnitZ();

    Eigen::Vector3d axis = now_normal.cross(target_normal).normalized();
    double angle = std::acos(now_normal.dot(target_normal));
    Eigen::Matrix3d rot = Eigen::AngleAxisd(angle, axis).toRotationMatrix();

    Eigen::Vector3d v0_pos = V.row(v0);
    Eigen::Vector3d v1_pos = V.row(v1);
    Eigen::Vector3d v2_pos = V.row(v2);

    // Rotate the triangle to the XY plane
    Eigen::Vector3d v0_pos_rot = rot * v0_pos;
    Eigen::Vector3d v1_pos_rot = rot * v1_pos;
    Eigen::Vector3d v2_pos_rot = rot * v2_pos;

    Eigen::Vector3d v0_pos_uv(V_UV(v0, 0), V_UV(v0, 1), 0);
    Eigen::Vector3d v1_pos_uv(V_UV(v1, 0), V_UV(v1, 1), 0);

    // Rotate the triangle inside the XY plane
    Eigen::Vector3d vuv = v1_pos_uv - v0_pos_uv;
    Eigen::Vector3d v = v1_pos_rot - v0_pos_rot;

    assert(v(2) <= 10e-3);

    Eigen::Vector3d axis_rot = vuv.cross(v).normalized();
    double angle_rot = std::acos(vuv.dot(v) / (vuv.norm() * v.norm()));
    Eigen::Matrix3d rot_rot = Eigen::AngleAxisd(angle_rot, axis_rot).toRotationMatrix();

    Eigen::Vector3d v2_pos_rot_rot = rot_rot * v2_pos_rot - (rot_rot * v0_pos_rot - v0_pos_uv);
    V_UV(v2, 0) = v2_pos_rot_rot(0);
    V_UV(v2, 1) = v2_pos_rot_rot(1);

    // Recursively apply this
    for(int i=0; i<3; i++){
        Eigen::Index new_Fid = TT(Fid, v2_F);

        if(new_Fid != -1){
            Eigen::Index new_v0_F = TTi(Fid, v2_F);
            Eigen::Index new_v1_F = (new_v0_F + 1) % 3;
            Eigen::Index new_v2_F = (new_v0_F + 2) % 3;
            fit_triangle_in_2d(V, F, N, TT, TTi, new_Fid, new_v0_F, new_v1_F, new_v2_F, V_UV, visited);
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
    Eigen::VectorXi visited = Eigen::VectorXi::Zero(V.rows());

    // Flatten the first triangle
    Eigen::Index Fid = 0;

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
    Eigen::Vector3d v0_pos_rot = rot * v0_pos;
    Eigen::Vector3d v1_pos_rot = rot * v1_pos;
    Eigen::Vector3d v2_pos_rot = rot * v2_pos;

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
            fit_triangle_in_2d(V, F, N, TT, TTi, new_Fid, new_v0_F, new_v1_F, new_v2_F, V_UV, visited);
        }
    }
}

#endif //NEBUTA_DESIGNER_FLATTEN_TRIANGLE_STRIP_H
