#ifndef TRIANGLE_STRIPS_H
#define TRIANGLE_STRIPS_H
#include <Eigen/Core>
#include <vector>
#include <utility>
#include <igl/point_mesh_squared_distance.h>
#include <igl/boundary_loop.h>
#include <iostream>

namespace tri_strip{

    class TriangleStrip{
        public:
        bool is_valid;
        double cost;
        size_t next_vert;

        TriangleStrip(bool is_valid, double cost, Eigen::Index next_vert){
            this->is_valid = is_valid;
            this->cost = cost;
            this->next_vert = next_vert;
        }

        TriangleStrip(){
            this->is_valid = false;
            this->cost = 0;
            this->next_vert = -1;
        }
    };

    void precompute_distance(Eigen::MatrixXd &V, 
                            Eigen::MatrixXi &F,
                            igl::AABB<Eigen::MatrixXd, 3> tree,
                            std::vector<Eigen::Index> &bnd_loop, 
                            std::vector<std::vector<double>> &distance_memo)
    {
        Eigen::MatrixXd P;
        Eigen::VectorXd sqrD;
        Eigen::VectorXi I;
        Eigen::MatrixXd C;

        for(int i=0; i<bnd_loop.size()-1; i++){
            P.resize(bnd_loop.size()-i-1, 3);
            P.setZero();

            for(int j=i+1; j<bnd_loop.size(); j++){
                P.row(j) = (V.row(bnd_loop[i]) + V.row(bnd_loop[j])) / 2;
            }

            tree.squared_distance(V, F, P, sqrD, I, C);

            for(int j=i+1; j<bnd_loop.size(); j++){
                distance_memo[i][j] = sqrD(j);
                distance_memo[j][i] = sqrD(j);
            }
        }
    }

    using triangle_strip_quality_mode = enum {per_face_distance, normal_alignment};

    void fill_triangles_loop(Eigen::MatrixXd &V, 
                            Eigen::MatrixXi &F,
                            igl::AABB<Eigen::MatrixXd, 3> tree,
                            std::vector<Eigen::Index> &bnd_loop, 
                            std::vector<std::vector<TriangleStrip>> &memo,
                            Eigen::MatrixXd &per_vert_normal,
                            triangle_strip_quality_mode mode = normal_alignment)
    {
        for(int i=0; i<bnd_loop.size(); i++){
            auto& now = memo[i][(i+1)%bnd_loop.size()];
            now.is_valid = true;
            now.cost = 0;
            now.next_vert = -1;
        }

        Eigen::MatrixXd P;
        Eigen::VectorXd energy_value;
        Eigen::VectorXi I;
        Eigen::MatrixXd C;

        P.resize(bnd_loop.size()-2, 3);

        for(int candidate_size=1; candidate_size<=bnd_loop.size()-2; candidate_size++){

            for(int start_vert_idx=0; start_vert_idx<bnd_loop.size(); start_vert_idx++){

                int end_vert_idx = (start_vert_idx + candidate_size + 1) % bnd_loop.size();

                if (mode == normal_alignment)
                {
                    energy_value.resize(candidate_size);
                }

                for(int candidate_idx=0; candidate_idx<candidate_size; candidate_idx++){

                    int target_idx = (start_vert_idx + candidate_idx + 1) % bnd_loop.size();

                    if(mode == per_face_distance) {
                        auto barycenter =
                                (V.row(bnd_loop[start_vert_idx]) + V.row(bnd_loop[target_idx]) + V.row(bnd_loop[end_vert_idx])) / 3;
                        P.row(candidate_idx) = barycenter;
                    }else if(mode == normal_alignment){
                        Eigen::Vector3d e1 = V.row(bnd_loop[target_idx]) - V.row(bnd_loop[start_vert_idx]);
                        Eigen::Vector3d e2 = V.row(bnd_loop[target_idx]) - V.row(bnd_loop[end_vert_idx]);
                        Eigen::VectorXd new_surface_normal = e1.cross(e2);
                        energy_value(candidate_idx) = 0;
                        energy_value(candidate_idx) += (new_surface_normal - Eigen::Vector3d(per_vert_normal.row(bnd_loop[start_vert_idx]))).norm();
                        energy_value(candidate_idx) += (new_surface_normal - Eigen::Vector3d(per_vert_normal.row(bnd_loop[end_vert_idx]))).norm();
                        energy_value(candidate_idx) += (new_surface_normal - Eigen::Vector3d(per_vert_normal.row(bnd_loop[target_idx]))).norm();
                    }
                }

                if(mode == per_face_distance) {
                    tree.squared_distance(V, F, P, energy_value, I, C);
                }

                double min_score = std::numeric_limits<double>::max();
                size_t next_vert = -1;

                for(int candidate_idx=0; candidate_idx<candidate_size; candidate_idx++){

                    int target_idx = (start_vert_idx + candidate_idx + 1) % bnd_loop.size();

                    auto start_vert = V.row(bnd_loop[start_vert_idx]);
                    auto target_vert = V.row(bnd_loop[target_idx]);
                    auto end_vert = V.row(bnd_loop[end_vert_idx]);

                    Eigen::Vector3d v1 = start_vert - target_vert;
                    Eigen::Vector3d v2 = end_vert - target_vert;

                    double area = v1.cross(v2).norm() / 2;
                    
                    double now_score = area * energy_value(candidate_idx);
                    now_score += memo[start_vert_idx][target_idx].cost;
                    now_score += memo[target_idx][end_vert_idx].cost;

                    assert(memo[start_vert_idx][target_idx].is_valid);
                    assert(memo[target_idx][end_vert_idx].is_valid);

                    if(now_score < min_score){
                        min_score = now_score;
                        next_vert = target_idx;
                    }
                }
                assert(next_vert != -1);
                memo[start_vert_idx][end_vert_idx].is_valid = true;
                memo[start_vert_idx][end_vert_idx].cost = min_score;
                memo[start_vert_idx][end_vert_idx].next_vert = next_vert;
            }
        }
    }

    void recursive_retrieve_faces(size_t first_idx, size_t second_idx, std::vector<Eigen::Index> &bnd_loop, std::vector<std::vector<TriangleStrip>> &memo, std::vector<Eigen::Vector3i> &faces){
        int candidate_size = (first_idx < second_idx)? (second_idx - first_idx - 1):(second_idx + bnd_loop.size() - first_idx - 1);
        if (candidate_size <= 0){
            return;
        }

        assert(memo[first_idx][second_idx].is_valid && memo[first_idx][second_idx].next_vert != -1);
        faces.push_back(Eigen::Vector3i(first_idx, memo[first_idx][second_idx].next_vert, second_idx));
        recursive_retrieve_faces(first_idx, memo[first_idx][second_idx].next_vert, bnd_loop, memo, faces);
        recursive_retrieve_faces(memo[first_idx][second_idx].next_vert, second_idx, bnd_loop, memo, faces);
    }

    void best_triangle_strip(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &resultV, Eigen::MatrixXi &resultF, std::vector<Eigen::Index> &bnd_loop, triangle_strip_quality_mode mode = normal_alignment){
        std::vector<std::vector<TriangleStrip>> memo(bnd_loop.size(), std::vector<TriangleStrip>(bnd_loop.size(), TriangleStrip()));
        igl::AABB<Eigen::MatrixXd, 3> tree;
        tree.init(V, F);

        //std::cerr << "vertices count: " << V.rows() << std::endl;
        //std::cerr << "boundary count: " << bnd_loop.size() << std::endl;

        Eigen::MatrixXd N;
        igl::per_vertex_normals(V, F, N);
        fill_triangles_loop(V, F, tree, bnd_loop, memo, N, mode);

        //std::cerr << "fill done" << std::endl;

        double min_cost = memo[1][0].cost;

        for(int i=0; i<bnd_loop.size(); i++){
            //std::cerr << memo[(i+1)%bnd_loop.size()][i].cost << std::endl;
        }

        std::vector<Eigen::Vector3i> faces;
        recursive_retrieve_faces(1, 0, bnd_loop, memo, faces);

        resultV.resize(bnd_loop.size(), 3);
        resultF.resize(faces.size(), 3);

        for(int i=0; i<bnd_loop.size(); i++){
            resultV.row(i) = V.row(bnd_loop[i]);
        }
        for(int i=0; i<faces.size(); i++){
            resultF.row(i) = faces[i];
        }
    }

    void approximate_single_patch(Eigen::MatrixXd &V, Eigen::MatrixXi &F, Eigen::MatrixXd &resultV, Eigen::MatrixXi &resultF, triangle_strip_quality_mode mode = normal_alignment){
        std::vector<Eigen::Index> bnd_loop;
        Eigen::VectorXi L;
        igl::boundary_loop(F, L);

        for(int i=0; i<L.size(); i++){
            bnd_loop.push_back(L(i));
        }
        best_triangle_strip(V, F, resultV, resultF, bnd_loop, mode);
    }
}
#endif