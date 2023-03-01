#include "developablize.h"

#include "./decimate.h"
#include "igl/per_vertex_point_to_plane_quadrics.h"
#include "igl/qslim_optimal_collapse_edge_callbacks.h"
#include "igl/quadric_binary_plus_operator.h"
//#include "./divide_borders.h"

#include <vector>
#include <stack>
#include <iostream>

#include <igl/boundary_loop.h>
#include <igl/decimate.h>
#include <igl/shortest_edge_and_midpoint.h>
#include <igl/edge_topology.h>
#include <igl/adjacency_list.h>

namespace rp_quadric {
    void setup_developablize(const Eigen::MatrixX3i &F, const Eigen::MatrixX3d &V,
                             const std::vector<std::vector<int>> &paths,
                             Eigen::MatrixX3i &resultF, Eigen::MatrixX3d &resultV,
                             std::vector<std::vector<int>> &_paths,
                             Eigen::VectorXi &pathIds,
                             Eigen::VectorXi &I, Eigen::VectorXi &J) {
        for (int i = 0; i < paths.size(); i++) {
            for (int j = 0; j < paths[i].size(); j++) {
                pathIds(paths[i][j]) = i;
            }
        }

        std::vector<std::vector<int>> boundaryLoops;
        igl::boundary_loop(F, boundaryLoops);

        for (int i = 0; i < boundaryLoops.size(); i++) {
            auto l = boundaryLoops[i];
            for (auto vid: l) {
                // paths.size()と被らないid
                pathIds(vid) = paths.size() + i;
            }
        }
        /*
        singularityOffset = paths.size() + boundaryLoops.size();

        int sc = 0;
        for (int i = 0; i < isSingularity.rows(); i++) {
          if (isSingularity(i)) {
            int pid = paths.size() + boundaryLoops.size() + sc;
            for (int j = 0; j < 3; j++) {
              pathIds(F(i, j)) = pid;
            }
            sc++;
          }
        }*/

        // 最後に、pathIdが-1でないものをVの先頭に持ってくる
        I.resize(V.rows());
        J.resize(V.rows());
        int cnt = 0;
        for (int i = 0; i < pathIds.rows(); i++) {
            if (pathIds(i) != -1) {
                J(cnt) = i;
                I(i) = cnt;
                cnt++;
            }
        }
        for (int i = 0; i < pathIds.rows(); i++) {
            if (pathIds(i) == -1) {
                J(cnt) = i;
                I(i) = cnt;
                cnt++;
            }
        }
        assert(cnt == pathIds.rows());

        resultF.resize(F.rows(), 3);
        resultV.resize(V.rows(), 3);

        for (int i = 0; i < V.rows(); i++) {
            resultV.row(i) = V.row(J(i));
        }
        for (int i = 0; i < F.rows(); i++) {
            for (int j = 0; j < 3; j++) {
                resultF(i, j) = I(F(i, j));
            }
        }

        auto _pathId = pathIds;
        for (int i = 0; i < V.rows(); i++) {
            _pathId(i) = pathIds(J(i));
        }
        pathIds = _pathId;

        _paths.resize(paths.size(), {});
        for (int i = 0; i < paths.size(); i++) {
            _paths[i].resize(paths[i].size(), -1);
            for (int j = 0; j < paths[i].size(); j++) {
                _paths[i][j] = I(paths[i][j]);
            }
        }
    }

    void setup_constrained_edges(const Eigen::MatrixXi &EV, const Eigen::VectorXi &V2NV,
                                 const std::vector<std::vector<int>> &paths,
                                 Eigen::VectorXi &isConstrainedEdge) {
        isConstrainedEdge.resize(EV.rows());
        isConstrainedEdge.setZero();

        std::map<std::pair<int, int>, int> map;
        for (int i = 0; i < EV.rows(); i++) {
            int v0 = EV(i, 0), v1 = EV(i, 1);
            if (v0 > v1) {
                std::swap(v0, v1);
            }

            map[std::pair<int, int>(v0, v1)] = i + 1;
        }

        for (auto path: paths) {
            for (int i = 1; i < path.size(); i++) {
                int v0 = V2NV(path[i - 1]);
                int v1 = V2NV(path[i]);

                // v0の方が常に小さい
                if (v0 > v1) {
                    std::swap(v0, v1);
                }

                if (map[std::pair<int, int>(v0, v1)] > 0) {
                    int eid = map[std::pair<int, int>(v0, v1)] - 1;
                    isConstrainedEdge(eid) = 1;
                } else {
                    assert(false && "no edge in map");
                }
            }
        }
    }

    void degenerate_internal_vertices(const Eigen::MatrixX3i &F, const Eigen::MatrixX3d &V,
                                      const Eigen::VectorXi &pathIds,
                                      Eigen::MatrixX3i &resultF, Eigen::MatrixX3d &resultV,
                                      Eigen::VectorXi &V2NV,
                                      Eigen::VectorXi &NV2V) {
        Eigen::MatrixXi _F, _resultF(resultF);
        Eigen::MatrixXd _V, _resultV(resultV);

        igl::connect_boundary_to_infinity(V, F, _V, _F);

        Eigen::VectorXi EMAP;
        Eigen::MatrixXi E,EF,EI;
        igl::edge_flaps(_F,E,EMAP,EF,EI);
        // Quadrics per vertex
        typedef std::tuple<Eigen::MatrixXd,Eigen::RowVectorXd,double> Quadric;
        std::vector<Quadric> quadrics;
        igl::per_vertex_point_to_plane_quadrics(_V,_F,EMAP,EF,EI,quadrics);
        // State variables keeping track of edge we just collapsed
        int v1 = -1;
        int v2 = -1;
        // Callbacks for computing and updating metric
        igl::decimate_cost_and_placement_callback cost_and_placement;
        igl::decimate_pre_collapse_callback       pre_collapse;
        igl::decimate_post_collapse_callback      post_collapse;
        igl::qslim_optimal_collapse_edge_callbacks(
                E,quadrics,v1,v2, cost_and_placement, pre_collapse,post_collapse);



        igl::decimate_cost_and_placement_callback qslim_cost_and_placement = [&quadrics, &v1, &v2, &pathIds](const int e,
                                                                                const Eigen::MatrixXd &V,
                                                                                const Eigen::MatrixXi & /*F*/,
                                                                                const Eigen::MatrixXi &E,
                                                                                const Eigen::VectorXi & /*EMAP*/,
                                                                                const Eigen::MatrixXi & /*EF*/,
                                                                                const Eigen::MatrixXi & /*EI*/,
                                                                                double &cost,
                                                                                Eigen::RowVectorXd &p) {
            Quadric quadric_p;
            quadric_p = igl::operator+(quadrics[E(e,0)], quadrics[E(e,1)]);
            // Quadric: p'Ap + 2b'p + c
            // optimal point: Ap = -b, or rather because we have row vectors: pA=-b
            const auto & A = std::get<0>(quadric_p);
            const auto & b = std::get<1>(quadric_p);
            const auto & c = std::get<2>(quadric_p);

            if (E(e, 0) >= pathIds.rows() || E(e, 1) >= pathIds.rows()) {
                int otherVid = E(e, 0) >= pathIds.rows() ? E(e, 1) : E(e, 0);
                if (otherVid < pathIds.rows() && pathIds(otherVid) != -1) {
                    // つぶさない
                    cost = INFINITY;
                    p = -b * A.inverse();
                } else {
                    p = -b * A.inverse();
                    cost = p.dot(p*A) + 2*p.dot(b) + c;
                }
            } else {
                if (pathIds(E(e, 0)) == -1 && pathIds(E(e, 1)) == -1) {
                    // 両方-1
                    // 当然つぶす
                    p = -b * A.inverse();
                    cost = p.dot(p*A) + 2*p.dot(b) + c;
                } else if (pathIds(E(e, 0)) == -1 || pathIds(E(e, 1)) == -1) {
                    // 片方-1
                    // つぶすが、pointは-1でない側に持ってくる
                    int vid = pathIds(E(e, 0)) == -1 ? E(e, 1) : E(e, 0);
                    p = V.row(vid);
                    cost = p.dot(p*A) + 2*p.dot(b) + c;
                } else {
                    // つぶさない
                    cost = INFINITY;
                    p = -b * A.inverse();
                }
            }
        };

        int m = _F.rows();
        int max = _F.rows();
        for (int i = 0; i < pathIds.rows(); i++) {
            if (pathIds(i) == -1) {
                max -= 2;
            }
        }

        igl::decimate_stopping_condition_callback maxM = igl::max_faces_stopping_condition(m, F.rows(), max);

        Eigen::VectorXi _;
        rp::decimate(_V, _F, qslim_cost_and_placement, maxM, pre_collapse, post_collapse, _resultV, _resultF, _, NV2V);

        resultF = _resultF;
        resultV = _resultV;

        // 最後のvertexとそれに関わるやつらをすべて消す
        resultV.conservativeResize(resultV.rows() - 1, 3);
        int cnt = 0;
        for (int i = 0; i < resultF.rows(); i++) {
            bool unnecessary = false;
            for (int j = 0; j < 3; j++) {
                if (resultF(i, j) >= resultV.rows()) {
                    unnecessary = true;
                    break;
                }

                if (resultF(i, j) == resultF(i, (j + 1) % 3)) {
                    unnecessary = true;
                    break;
                }
            }

            if (!unnecessary) {
                resultF.row(cnt) = resultF.row(i);
                cnt++;
            }
        }

        resultF.conservativeResize(cnt, 3);
        NV2V.conservativeResize(NV2V.rows() - 1);
        V2NV.resize(V.rows());
        V2NV.setConstant(-1);
        for (int i = 0; i < NV2V.rows(); i++) {
            V2NV(NV2V(i)) = i;
        }

        assert(resultF.maxCoeff() == resultV.rows() - 1);
    }

    void flip_edge(const int &edgeId,
                   Eigen::MatrixX3i &F, Eigen::MatrixXi &EF,
                   Eigen::MatrixXi &FE, Eigen::MatrixXi &EV, std::vector<int> &nextEids) {
        nextEids = {};

        auto faces = EF.row(edgeId);
        auto flippingVertices = EV.row(edgeId);
        std::vector<int> otherVids = {};

        for (int i = 0; i < 2; i++) {
            int fid = faces(i);
            for (int j = 0; j < 3; j++) {
                if (F(fid, j) != flippingVertices(0) && F(fid, j) != flippingVertices(1)) {
                    otherVids.emplace_back(F(fid, j));
                }
            }
        }

        if (otherVids.size() != 2) {
            std::cout << F.row(faces(0)) << std::endl;
            std::cout << F.row(faces(1)) << std::endl;
            for (auto v: otherVids) {
                std::cout << v << ", ";
            }
            std::cout << std::endl;
        }

        assert(otherVids.size() == 2);

        // nextEidsを入れる
        for (int i = 0; i < 2; i++) {
            int fid = faces(i);
            for (int j = 0; j < 3; j++) {
                if (FE(fid, j) == edgeId) {
                    nextEids.emplace_back(FE(fid, (j + 1) % 3));
                    nextEids.emplace_back(FE(fid, (j + 2) % 3));
                    break;
                }
            }
        }

        if (nextEids.size() != 4) {
            std::cout << FE.row(faces(0)) << std::endl;
            std::cout << FE.row(faces(1)) << std::endl;
            std::cout << nextEids.size() << ", " << edgeId << std::endl;
            assert(nextEids.size() == 4);
        }

        // ベンツのロゴみたいになってる頂点はダメ
        for (int i = 0; i < 2; i++) {
            for (int j = 0; j < 2; j++) {
                int e0 = nextEids[i], e1 = nextEids[j + 2];
                if (EF(e0, 0) == EF(e1, 0) || EF(e0, 0) == EF(e1, 1) || EF(e0, 1) == EF(e1, 0) ||
                    EF(e0, 1) == EF(e1, 1)) {
                    nextEids = {};
                    return;
                }
            }
        }

        for (int i = 0; i < nextEids.size(); i++) {
            // -1になってるのもだめ
            if (EF(nextEids[i], 0) == -1 || EF(nextEids[i], 1) == -1) {
                nextEids = {};
                return;
            }
        }

        // FEを入れ替え
        FE.row(faces(0)) << nextEids[0], edgeId, nextEids[3];
        FE.row(faces(1)) << nextEids[2], edgeId, nextEids[1];

        // EFを入れ替え
        for (int i = 0; i < 2; i++) {
            int idx = 2 * i + 1;
            for (int j = 0; j < 2; j++) {
                if (EF(nextEids[idx], j) == faces(i)) {
                    EF(nextEids[idx], j) = faces((i + 1) % 2);
                    break;
                }
                assert(j != 1);
            }
        }

        // Fを入れ替え
        F.row(faces(0)) << otherVids[0], otherVids[1], -1;
        F.row(faces(1)) << otherVids[1], otherVids[0], -1;
        for (int k = 0; k < 2; k++) {
            for (int l = 0; l < 2; l++) {
                if (EV(nextEids[0], k) == EV(nextEids[1], l)) {
                    F(faces(0), 2) = EV(nextEids[0], (k + 1) % 2);
                    F(faces(1), 2) = EV(nextEids[1], (l + 1) % 2);
                }
            }
        }

        // EVを書き換え
        for (int i = 0; i < 2; i++) {
            EV(edgeId, i) = i == 0 ? std::min(otherVids[0], otherVids[1]) : std::max(otherVids[0], otherVids[1]);
        }

    }

    bool needs_inversion(const Eigen::MatrixX3i &F, const Eigen::MatrixXi &EF,
                         const Eigen::MatrixXi &FE,
                         const Eigen::MatrixXi &EV, const Eigen::VectorXi &NV2V,
            /*const Eigen::VectorXd &params,*/ const int &edgeId,
                         const Eigen::VectorXi &pathIds, const bool &useDiff) {
        std::vector<int> otherVids = {};

        std::vector<std::vector<int>> otherEdgeIds = {{},
                                                      {}};

        for (int j = 0; j < 2; j++) {
            int fid = EF(edgeId, j);
            if (fid == -1) {
                return false;
            }

            for (int k = 0; k < 3; k++) {
                if (EV(edgeId, 0) != F(fid, k) && EV(edgeId, 1) != F(fid, k)) {
                    otherVids.emplace_back(F(fid, k));
                }

                if (FE(fid, k) != edgeId) {
                    otherEdgeIds[j].emplace_back(FE(fid, k));
                }
            }
        }

        for (int j = 0; j < 2; j++) {
            for (int k = 0; k < 2; k++) {
                int e0 = otherEdgeIds[0][j], e1 = otherEdgeIds[1][k];
                // ベンツのロゴみたいになってるのでinvertしない
                if (EF(e0, 0) == EF(e1, 0) || EF(e0, 0) == EF(e1, 1)) {
                    if (EF(e0, 0) != EF(edgeId, 0) && EF(e0, 0) != EF(edgeId, 1)) {
                        return false;
                    }
                } else if (EF(e0, 1) == EF(e1, 0) || EF(e0, 1) == EF(e1, 1)) {
                    if (EF(e0, 1) != EF(edgeId, 0) && EF(e0, 1) != EF(edgeId, 1)) {
                        return false;
                    }
                }
            }
        }

        if (otherVids.size() == 2) {
            // 対辺が2つの端をつなぐものになれば必ず入れ替える
            if (pathIds(NV2V(otherVids[0])) != pathIds(NV2V(otherVids[1])) &&
                pathIds(NV2V(EV(edgeId, 0))) == pathIds(NV2V(EV(edgeId, 1)))) {

                return true;
            } else if (pathIds(NV2V(EV(edgeId, 0))) != pathIds(NV2V(EV(edgeId, 1))) &&
                       pathIds(NV2V(otherVids[0])) == pathIds(NV2V(otherVids[1]))) {
                // 入れ替えたら上みたいになるのは弾いておく
                return false;
            }
        }

        // デフォルトは入れ替え不要
        return false;
    }

    void edge_inversion(/*const Eigen::VectorXd &zeroFormV,*/ const Eigen::VectorXi &NV2V,
                                                              const Eigen::VectorXi &pathIds, const bool &useDiff,
                                                              const Eigen::VectorXi &isConstrainedEdge,
                                                              Eigen::MatrixXi &EV, Eigen::MatrixXi &EF,
                                                              Eigen::MatrixXi &FE, Eigen::MatrixX3i &resultF) {
        std::map<int, bool> violatingEdgeLists;
        for (int i = 0; i < EV.rows(); i++) {
            if (needs_inversion(resultF, EF, FE, EV, NV2V, /*zeroFormV,*/ i, pathIds, useDiff) &&
                !isConstrainedEdge(i)) {
                violatingEdgeLists[i] = true;
            }
        }

        while (violatingEdgeLists.size() > 0) {
            int initEdgeId = violatingEdgeLists.begin()->first;
            std::stack<int> stack;
            stack.push(initEdgeId);

            while (!stack.empty()) {
                int eid = stack.top();
                stack.pop();

                if (needs_inversion(resultF, EF, FE, EV, NV2V, /*zeroFormV,*/ eid, pathIds, useDiff)) {
                    if (isConstrainedEdge(eid)) {
                        continue;
                    }

                    std::vector<int> nextEids;
                    flip_edge(eid, resultF, EF, FE, EV, nextEids);

                    for (auto ne: nextEids) {
                        if (!isConstrainedEdge(ne)) {
                            stack.push(ne);
                        }
                    }
                }

                // 入れ替えたら消す
                violatingEdgeLists.erase(eid);
            }
        }
    }


    void developablize(const Eigen::MatrixX3i &F, const Eigen::MatrixX3d &V,
                       const std::vector<std::vector<int>> &paths,
            // ある面が特異点かどうか
            /*const Eigen::VectorXi &isSingularity, const Eigen::VectorXd &zeroFormV,*/
                       Eigen::MatrixX3i &resultF, Eigen::MatrixX3d &resultV) {
        Eigen::VectorXi pathIds(V.rows());
        pathIds.setConstant(-1);
        int singularityOffset;

        Eigen::MatrixX3i _F;
        Eigen::MatrixX3d _V;
        Eigen::VectorXi I, J;
        std::vector<std::vector<int>> _paths;

        setup_developablize(F, V, paths, /*isSingularity,*/ _F, _V, _paths, /*singularityOffset,*/ pathIds, I, J);

        assert(igl::is_edge_manifold(_F));

        Eigen::VectorXi V2NV, NV2V;
        degenerate_internal_vertices(_F, _V, pathIds, resultF, resultV, V2NV, NV2V);

        assert(igl::is_edge_manifold(_F));

        Eigen::MatrixXi EF, EV, FE;
        igl::edge_topology(resultV, resultF, EV, FE, EF);

        Eigen::VectorXi isConstrainedEdge(EV.rows());
        isConstrainedEdge.setZero();
        setup_constrained_edges(EV, V2NV, _paths, isConstrainedEdge);

        // std::cout << "constrained edges: " << isConstrainedEdge.sum() << std::endl;

        // edge_inversion(_zeroFormV, NV2V, pathIds, false, isConstrainedEdge, EV, EF, FE, resultF);
        edge_inversion(/*_zeroFormV,*/ NV2V, pathIds, false, isConstrainedEdge, EV, EF, FE, resultF);

        int count = 0;
        for (int i = 0; i < NV2V.rows(); i++) {
            if (NV2V(i) < pathIds.rows() && pathIds(NV2V(i)) == -1) {
                count++;
            }
        }

        std::vector<std::vector<int>> newPathVids;
        newPathVids.resize(_paths.size(), {});
        for (int i = 0; i < _paths.size(); i++) {
            newPathVids[i].resize(_paths[i].size());
            for (int j = 0; j < _paths[i].size(); j++) {
                newPathVids[i][j] = V2NV(_paths[i][j]);
            }
        }

        // divide_border(resultV, resultF, newPathVids, pathIds);
        // rp::divide_borders::process(resultV, resultF, newPathVids);
        std::cout << "non degenerated vertices: " << count << std::endl;

        assert(count == 0);
    }

    void approximate_single_patch(const Eigen::MatrixXd &V, const Eigen::MatrixXi &F,
                                  Eigen::MatrixXd &resultV, Eigen::MatrixXi &resultF) {
        std::vector <std::vector<int>> boundary_loop;
        igl::boundary_loop(F, boundary_loop);

        Eigen::MatrixX3d _V = V;
        Eigen::MatrixX3i _F = F;
        Eigen::MatrixX3d _resultV;
        Eigen::MatrixX3i _resultF;

        developablize(_F, _V, boundary_loop, _resultF, _resultV);

        resultV = _resultV;
        resultF = _resultF;
    }

}
