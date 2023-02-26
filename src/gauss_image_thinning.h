#ifndef GAUSS_IMAGE_THINNING_H
#define GAUSS_IMAGE_THINNING_H

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "igl/boundary_loop.h"
#include "igl/per_vertex_normals.h"
#include "igl/cotmatrix.h"
#include "igl/cotmatrix_entries.h"
#include "igl/adjacency_list.h"
#include "igl/triangle_triangle_adjacency.h"
#include "igl/barycenter.h"
#include "igl/massmatrix.h"
#include "igl/writeOBJ.h"
#include "igl/edge_topology.h"
#include <fstream>
#include <cmath>
#include <array>

void fitNormalsSinglePatch(const Eigen::MatrixXd& V,
                           const Eigen::MatrixXd& N,
                           Eigen::MatrixXd& N2)
{
    const auto nv = N.rows();
    N2.resize(nv, 3);
    
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(N.transpose() * N, Eigen::ComputeFullV);
    Eigen::Matrix3d frame = svd.matrixV();

    for(int i=0; i<nv; i++){
        N2.row(i) = ((frame.leftCols(2) * frame.leftCols(2).transpose() * N.row(i).transpose()).normalized());
    }
}            


#endif
