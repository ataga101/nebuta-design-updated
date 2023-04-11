#ifndef TRIANGLE_STRIPS_H
#define TRIANGLE_STRIPS_H
#include <Eigen/Core>
#include <vector>
#include <utility>
#include <igl/point_mesh_squared_distance.h>
#include <igl/boundary_loop.h>
#include <igl/per_vertex_normals.h>
#include <iostream>

namespace tri_strip{
    using triangle_strip_quality_mode = enum {per_face_distance, normal_alignment};
    void approximate_single_patch(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &resultV, Eigen::MatrixXi &resultF, triangle_strip_quality_mode mode = normal_alignment);
}
#endif