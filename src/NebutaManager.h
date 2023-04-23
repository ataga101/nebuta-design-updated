//
// Created by Naoki Agata on 2023/02/23.
//

#ifndef LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H
#define LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H

#include <Eigen/Core>
#include <string>
#include "fields/field_smoother.h"
#include "mesh_field_smoother.h"
#include "mesh_quality.h"
#include "triangle_mesh_type.h"
#include "poly_mesh_type.h"
#include "AutoRemesher.h"
#include "tracing/mesh_type.h"
#include "tracing/patch_tracer.h"
#include "tracing/tracer_interface.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

#include "mesh_manager.h"
#include "svg_writer.h"
#include "flatten_triangle_strip.h"

#include "igl/read_triangle_mesh.h"
#include "igl/remove_duplicate_vertices.h"

#include "vcg/complex/algorithms/create/platonic.h"
#include "cstdio"
#include "approximate_single_patch.h"

#include "igl/hausdorff.h"
#include "igl/combine.h"

using TracerType = PatchTracer<TraceMesh, MeshQuality<TraceMesh>>;
using FieldSmootherType = vcg::tri::FieldSmoother<FieldTriMesh>;

class NebutaManager {
public:
    enum display_mode {ORIGINAL, REMESHED, PARTITIONED, APPROXIMATED, FABRICATIONMODE, WIREONLY};
private:
    FieldTriMesh originalVCGMesh;

    FieldSmootherType::SmoothParam FieldParam;

    TraceMesh traceMesh;
    VertexFieldGraph<TraceMesh>* VGraph;
    TracerType* PTr;

    std::string pathM;

    polyscope::SurfaceMesh* polyscopeOriginalMesh = nullptr;
    polyscope::SurfaceMesh* polyscopeRemeshedMesh = nullptr;
    polyscope::SurfaceMesh* polyscopePartitionedMesh = nullptr;
    polyscope::SurfaceMesh* polyscopeApproximatedMesh = nullptr;
    polyscope::SurfaceMesh* polyscopeFlattenedMesh = nullptr;
    polyscope::SurfaceMesh* polyscopePatchMesh = nullptr;
    polyscope::SurfaceMesh* polyscopeWireMesh = nullptr;

    display_mode current_mode = ORIGINAL;
    ParamMode quality_measure = PMHausdorff;

    bool remeshed = false;
    bool partitioned = false;
    bool approximated = false;

    // Wire graph infos
    std::vector<Eigen::Vector3d> wirePos;
    std::vector<std::array<size_t, 2>> edgeInds;

    void set_parameters();
    
    // patch_idx to face_idx mapping (used for coloring)
    std::vector<int> patch_to_max_vert_idx;
    std::vector<int> patch_to_max_face_idx;

    Eigen::MatrixXd height_candidates;

public:
    Eigen::MatrixXd originalV;
    Eigen::MatrixXi originalF;
    Eigen::MatrixXd remeshedV;
    Eigen::MatrixXi remeshedF;
    Eigen::MatrixXd partitionedV;
    Eigen::MatrixXi partitionedF;
    Eigen::MatrixXd approximatedV;
    Eigen::MatrixXi approximatedF;

    double Drift = 100;
    double sample_ratio = 1;
    double height_in_cm = 14.1;
    bool split_on_removal = true;
    bool CClarkability = false;
    bool match_valence = false;
    bool check_quality_functor = true;
    bool add_only_needed = true;
    bool final_removal = true;
    bool meta_mesh_collapse = false;
    bool force_split = false;
    bool allow_trace_loopish = true;

    bool visualize_crease = false;

    int fabrication_mode_patch_id;

    std::vector<Eigen::MatrixXd> flattenedVs;
    std::vector<Eigen::MatrixXi> flattenedFs;
    std::vector<Eigen::MatrixXd> approximatedVs;
    std::vector<Eigen::MatrixXd> patchesVs;
    std::vector<Eigen::MatrixXi> patchesFs;

    NebutaManager();
    ~NebutaManager();
    void load_mesh(std::string mesh_filename);
    void remesh_and_field_computation();
    void init_structures();
    void prepare_tracing();
    void patch_tracing();
    void developable_approximation();

    double get_wire_length();
    int get_wire_cnt();
    double get_hausdorff_distance();
    int get_patch_cnt();
    int get_patch_quality_mode();

    void set_visualization_mode(display_mode mode);
    void color_selected_patch(Eigen::MatrixXd &per_face_color, int patch_idx);
    void set_patch_quality_mode(ParamMode mode);
    void set_threshold(ParamMode mode, double th);
    void update_visualization();

    void save_approximated_mesh();
    void save_wire_indices();
    void save_wire_positions();
    void save_2d_pattern();

    void binary_search_param(ParamMode mode, double target_patch_cnt);
    void do_test();
};

//
// Created by Naoki Agata on 2023/02/23.
//


NebutaManager::NebutaManager()
        : VGraph(nullptr), PTr(nullptr)
{

}

NebutaManager::~NebutaManager() {
    save_wire_indices();
    save_wire_positions();
    if(VGraph != nullptr) delete VGraph;
    if(PTr != nullptr) delete PTr;
    std::remove((pathM + "_field.rosy").c_str());
}

void NebutaManager::set_threshold(ParamMode mode, double th){
    if(mode == PMArap){
        MeshQuality<TraceMesh>::MinQ() = -th;
        MeshQuality<TraceMesh>::MaxQ() = th;
    }else if(mode == PMGaussImageThinness){
        MeshQuality<TraceMesh>::MaxGaussImageThickness() = th;
    }else if(mode == PMGaussianCurvature){
        MeshQuality<TraceMesh>::MaxSumOfGaussianCurvature() = th;
    }else{
        MeshQuality<TraceMesh>::MaxHausdorff() = th;
    }
}

void NebutaManager::binary_search_param(ParamMode mode, double hausdorff_distance){
    double max_th;
    double min_th = 0;
    if(mode == PMArap){
        max_th = 0.2;
    }else if(mode == PMGaussImageThinness){
        max_th = 0.002;
    }else if(mode == PMGaussianCurvature){
        max_th = 4;
    }else if(mode == PMHausdorff){
        max_th = 0.1;
    }else{
        std::cerr << "Not Implemented" << std::endl;
        return;
    }

    double mid;
    int max_trial_times = 20;
    do{
        std::cerr << "Binary Search: " << max_trial_times << " times left" << std::endl;
        mid = (max_th + min_th) / 2;
        prepare_tracing();
        set_threshold(mode, mid);
        set_patch_quality_mode(mode);
        patch_tracing();
        developable_approximation();
        if(get_hausdorff_distance() < hausdorff_distance){
            min_th = mid;
        }
        else{
            max_th = mid;
        }
        max_trial_times--;
    }while(std::abs(get_patch_cnt() - hausdorff_distance) > 10e-3 && max_trial_times > 0);

}

void NebutaManager::do_test() {
    std::ofstream ofs(pathM + "_test_res.txt");

    /*
    ofs << "Gauss Image Thinness: " << std::endl;
    std::cerr << "Testing Gauss Image Thinness" << std::endl;
    set_patch_quality_mode(PMGaussImageThinness);
    prepare_tracing();
    patch_tracing();
    developable_approximation();
    ofs << "patch_cnt: " << get_patch_cnt() << std::endl;
    ofs << "wire_cnt: " << get_wire_cnt() << std::endl;
    ofs << "wire_length: " << get_wire_length() << std::endl;
    ofs << "hausdorff_distance: " << get_hausdorff_distance() << std::endl;
    ofs << "Threshold: " << MeshQuality<TraceMesh>::MaxGaussImageThickness() << std::endl;

    ofs << "Gaussian Curvature: " << std::endl;
    std::cerr << "Testing Gaussian Curvature" << std::endl;
    binary_search_param(PMGaussianCurvature, get_patch_cnt());
    ofs << "patch_cnt: " << get_patch_cnt() << std::endl;
    ofs << "wire_cnt: " << get_wire_cnt() << std::endl;
    ofs << "wire_length: " << get_wire_length() << std::endl;
    ofs << "hausdorff_distance: " << get_hausdorff_distance() << std::endl;
    ofs << "Threshold: " << MeshQuality<TraceMesh>::MaxSumOfGaussianCurvature() << std::endl;
    */

    double hd_target = 0.025;

    approximate_single_patch::approx_mode = approximate_single_patch::QSlim;
    ofs << "Hausdorff: " << std::endl;
    std::cerr << "Testing Hausdorff" << std::endl;
    binary_search_param(PMHausdorff, hd_target);
    ofs << "patch_cnt: " << get_patch_cnt() << std::endl;
    ofs << "wire_cnt: " << get_wire_cnt() << std::endl;
    ofs << "wire_length: " << get_wire_length() << std::endl;
    ofs << "hausdorff_distance: " << get_hausdorff_distance() << std::endl;
    ofs << "Threshold: " << MeshQuality<TraceMesh>::MaxHausdorff() << std::endl;

    approximate_single_patch::approx_mode = approximate_single_patch::DP_perface_distance;
    ofs << "Hausdorff (DP): " << std::endl;
    std::cerr << "Testing Hausdorff (DP)" << std::endl;
    binary_search_param(PMHausdorff, hd_target);
    ofs << "patch_cnt: " << get_patch_cnt() << std::endl;
    ofs << "wire_cnt: " << get_wire_cnt() << std::endl;
    ofs << "wire_length: " << get_wire_length() << std::endl;
    ofs << "hausdorff_distance: " << get_hausdorff_distance() << std::endl;
    ofs << "Threshold: " << MeshQuality<TraceMesh>::MaxHausdorff() << std::endl;

    approximate_single_patch::approx_mode = approximate_single_patch::QSlim;
    ofs << "Arap: " << std::endl;
    std::cerr << "Testing Arap" << std::endl;
    binary_search_param(PMArap, hd_target);
    ofs << "patch_cnt: " << get_patch_cnt() << std::endl;
    ofs << "wire_cnt: " << get_wire_cnt() << std::endl;
    ofs << "wire_length: " << get_wire_length() << std::endl;
    ofs << "hausdorff_distance: " << get_hausdorff_distance() << std::endl;
    ofs << "Threshold: " << MeshQuality<TraceMesh>::MaxQ() << std::endl;

    approximate_single_patch::approx_mode = approximate_single_patch::DP_perface_distance;
    ofs << "Arap (DP): " << std::endl;
    std::cerr << "Testing Arap (DP)" << std::endl;
    binary_search_param(PMArap, hd_target);
    ofs << "patch_cnt: " << get_patch_cnt() << std::endl;
    ofs << "wire_cnt: " << get_wire_cnt() << std::endl;
    ofs << "wire_length: " << get_wire_length() << std::endl;
    ofs << "hausdorff_distance: " << get_hausdorff_distance() << std::endl;
    ofs << "Threshold: " << MeshQuality<TraceMesh>::MaxQ() << std::endl;

    ofs.close();

    std::cerr << "Test Done" << std::endl;
}

void NebutaManager::set_visualization_mode(display_mode mode) {
    current_mode = mode;
}

void NebutaManager::set_patch_quality_mode(ParamMode mode) {
    quality_measure = mode;
}


template<typename MeshType>
void ConvertToVCGMesh(MeshType &m, Eigen::MatrixXd &V, Eigen::MatrixXi &F){

    std::vector<std::vector<int>> vec_OF;
    std::vector<std::vector<double>> vec_OV;
    for(int i=0; i<F.rows(); i++){
        vec_OF.push_back(std::vector<int>{F(i, 0), F(i, 1), F(i, 2)});
    }
    for(int i=0; i<V.rows(); i++){
        vec_OV.push_back(std::vector<double>{V(i, 0), V(i, 1), V(i, 2)});
    }
    vcg::tri::BuildMeshFromCoordVectorIndexVector(m, vec_OV, vec_OF);
}

void NebutaManager::load_mesh(std::string filename) {
    // Load a mesh
    igl::read_triangle_mesh(filename, originalV, originalF);

    // Rescale the mesh
    height_candidates = (originalV.colwise().maxCoeff() - originalV.colwise().minCoeff());
    double max_height = height_candidates.maxCoeff();
    std::cerr << "Max height: " << max_height << std::endl;
    originalV /= max_height;

    // Convert to VCG mesh
    ConvertToVCGMesh(originalVCGMesh, originalV, originalF);
    originalVCGMesh.UpdateDataStructures();

    // Get the path of the mesh
    auto dot_pos = filename.find_first_of('.');
    pathM = filename.substr(0, dot_pos);

    // Register to polyscope
    polyscopeOriginalMesh = polyscope::registerSurfaceMesh("Original Mesh", originalV, originalF);
    polyscopeOriginalMesh->setSurfaceColor({0.9, 0.9, 0.9});
    polyscopeOriginalMesh->setEdgeWidth(1.);
    current_mode = ORIGINAL;
}

void NebutaManager::remesh_and_field_computation() {
    if(remeshed){
        current_mode = REMESHED;
        return;
    }

    typename MeshPrepocess<FieldTriMesh>::BatchParam BPar;
    BPar.DoRemesh=true;
    BPar.UpdateSharp= true;
    MeshPrepocess<FieldTriMesh>::BatchProcess(originalVCGMesh, BPar, FieldParam);

    // Save the temporal RoSy Field to file
    originalVCGMesh.SaveField(pathM + "_field.rosy");

    // Store remeshed mesh in igl format
    vcg::tri::MeshToMatrix<FieldTriMesh>::GetTriMeshData(originalVCGMesh, remeshedF, remeshedV);

    // Register to polyscope
    polyscopeRemeshedMesh = polyscope::registerSurfaceMesh("Remeshed Mesh", remeshedV, remeshedF);
    polyscopeRemeshedMesh->setSurfaceColor({0.9, 0.9, 0.9});
    polyscopeRemeshedMesh->setEdgeWidth(1.);
    current_mode = REMESHED;
    remeshed = true;
}

void NebutaManager::set_parameters() {
    double s = traceMesh.bbox.Diag();
    MeshQuality<TraceMesh>::AreaScale() = s * s;

    PTr->Drift = Drift;
    PTr->sample_ratio = sample_ratio;
    PTr->split_on_removal = split_on_removal;
    PTr->CClarkability = CClarkability;
    PTr->match_valence = match_valence;
    PTr->check_quality_functor = check_quality_functor;
    PTr->AllowTraceLoopish = allow_trace_loopish;

    MeshQuality<TraceMesh>::UVMode() = quality_measure;
}

void NebutaManager::patch_tracing() {
    if(!remeshed){
        remesh_and_field_computation();
    }

    // Set parameters
    set_parameters();

    // Trace patches
    RecursiveProcess<TracerType>(*PTr,Drift, add_only_needed,final_removal,true,meta_mesh_collapse,force_split);
    PTr->SmoothPatches();

    // Store partitioned mesh in igl format
    vcg::tri::MeshToMatrix<TraceMesh>::GetTriMeshData(PTr->Mesh(), partitionedF, partitionedV);

    // Register to polyscope
    polyscopePartitionedMesh = polyscope::registerSurfaceMesh("Partitioned Mesh", partitionedV, partitionedF);
    polyscopePartitionedMesh->setSurfaceColor({0.9, 0.9, 0.9});
    polyscopePartitionedMesh->setEdgeWidth(1.);

    current_mode = PARTITIONED;

    // Draw wires
    std::vector<std::vector<size_t> > CurrV;
    std::vector<std::vector<size_t> > _CurrDir;
    std::vector<bool> IsLoop;

    wirePos.clear();
    edgeInds.clear();
    PTr->GetCurrVertDir(CurrV, _CurrDir, IsLoop);

    size_t start_idx = 0;

    for (size_t i = 0; i < CurrV.size(); i++) {
        int limit = IsLoop[i] ? CurrV[i].size() : CurrV[i].size() - 1;
        for (size_t j1 = 0; j1 < CurrV[i].size(); j1++) {
            size_t j2 = (j1 + 1) % CurrV[i].size();
            wirePos.push_back(partitionedV.row(CurrV[i][j1]));
            if(j1 < limit) {
                edgeInds.push_back({start_idx + j1, start_idx + j2});
            }
        }
        start_idx += CurrV[i].size();
    }

    polyscope::SurfaceGraphQuantity* showWires_partitioned = polyscopePartitionedMesh->addSurfaceGraphQuantity("wires", wirePos,edgeInds);
    showWires_partitioned->setEnabled(true);
    showWires_partitioned->setColor({0.0, 0.0, 0.0});

    polyscopeWireMesh = polyscope::registerSurfaceMesh("Wire Mesh", partitionedV, Eigen::MatrixXi(0, 3));
    polyscope::SurfaceGraphQuantity* showWires_only = polyscopeWireMesh->addSurfaceGraphQuantity("wires", wirePos,edgeInds);
    showWires_only->setEnabled(true);
    showWires_only->setColor({0.0, 0.0, 0.0});
    current_mode = PARTITIONED;
    partitioned = true;
}

void NebutaManager::developable_approximation() {
    if(!partitioned){
        prepare_tracing();
        patch_tracing();
    }

    flattenedVs.clear();
    flattenedFs.clear();
    approximatedVs.clear();

    Eigen::MatrixXd N, dblA;
    igl::per_face_normals(partitionedV, partitionedF, N);
    igl::doublearea(partitionedV, partitionedF, dblA);

    patch_to_max_vert_idx.clear();
    patch_to_max_vert_idx.resize(PTr->Partitions.size(), 0);
    patch_to_max_face_idx.clear();
    patch_to_max_face_idx.resize(PTr->Partitions.size(), 0);

    int approximatedV_size = 0;
    int approximatedF_size = 0;

    for(auto i=0; i<PTr->Partitions.size(); i++) {
        // Get the patch mesh's normal
        Eigen::Vector3d avg_patch_normal = Eigen::Vector3d::Zero();
        for (int j = 0; j < PTr->Partitions[i].size(); j++) {
            avg_patch_normal += N.row(PTr->Partitions[i][j]) * dblA(PTr->Partitions[i][j]);
        }
        avg_patch_normal.normalize();

        // Get the patch mesh
        TraceMesh patchMesh;
        PTr->GetPatchMesh(i, patchMesh, false);
        Eigen::MatrixXd patchV, resultPatchV;
        Eigen::MatrixXi patchF, resultPatchF;
        vcg::tri::MeshToMatrix<TraceMesh>::GetTriMeshData(patchMesh, patchF, patchV);

        approximate_single_patch::approximate_single_patch(patchV, patchF, resultPatchV, resultPatchF);

        approximatedVs.push_back(resultPatchV);
        patchesVs.push_back(patchV);
        patchesFs.push_back(patchF);

        // Flatten the mesh
        Eigen::MatrixXd flattenedPatchV;

        Eigen::VectorXi bnd;
        igl::boundary_loop(resultPatchF, bnd);

        Eigen::MatrixXd scaledV = resultPatchV * height_in_cm * height_candidates(2) / height_candidates.maxCoeff();
        flatten_triangle_strip(scaledV, resultPatchF, -avg_patch_normal, flattenedPatchV);

        auto min_x = flattenedPatchV.col(0).minCoeff();
        auto min_y = flattenedPatchV.col(1).minCoeff();
        auto max_x = flattenedPatchV.col(0).maxCoeff();
        auto max_y = flattenedPatchV.col(1).maxCoeff();

        if ((max_y - min_y) > (max_x - min_x)) {
            for(int j=0; j<flattenedPatchV.rows(); j++){
                auto swap = flattenedPatchV(j, 0);
                flattenedPatchV(j, 0) = flattenedPatchV(j, 1);
                flattenedPatchV(j, 1) = -swap;
            }
        }
        min_x = flattenedPatchV.col(0).minCoeff();
        min_y = flattenedPatchV.col(1).minCoeff();

        for (int j = 0; j < flattenedPatchV.rows(); j++) {
            flattenedPatchV(j, 0) -= min_x;
            flattenedPatchV(j, 1) -= min_y;
        }

        flattenedVs.push_back(flattenedPatchV);
        flattenedFs.push_back(resultPatchF);

        approximatedV_size += resultPatchV.rows();
        approximatedF_size += resultPatchF.rows();

        patch_to_max_vert_idx[i] = approximatedV_size;
        patch_to_max_face_idx[i] = approximatedF_size;
    }

    // Concatenate all the approximated meshes
    approximatedV.resize(approximatedV_size, 3);
    approximatedF.resize(approximatedF_size, 3);

    int approximatedV_idx = 0;
    for(int i=0; i<approximatedVs.size(); i++){
        for(int j=0; j<approximatedVs[i].rows(); j++){
            approximatedV.row(approximatedV_idx) = approximatedVs[i].row(j);
            approximatedV_idx++;
        }
    }

    int approximatedF_idx = 0;
    for(int i=0; i<flattenedFs.size(); i++){
        for(int j=0; j<flattenedFs[i].rows(); j++){
            for(int k=0; k<3; k++){
                approximatedF(approximatedF_idx, k) = (i == 0) ? flattenedFs[i](j, k) : flattenedFs[i](j, k) + patch_to_max_vert_idx[i-1];
            }
            approximatedF_idx++;
        }
    }

    Eigen::MatrixXd allpatchV;
    Eigen::MatrixXi allpatchF;

    igl::combine(patchesVs, patchesFs, allpatchV, allpatchF);

    igl::writeOBJ(pathM + "_allpatch.obj", allpatchV, allpatchF);

    // Register to polyscope
    polyscopeApproximatedMesh = polyscope::registerSurfaceMesh("Approximated Mesh", approximatedV, approximatedF);
    polyscopeApproximatedMesh->setEdgeWidth(1.);
    polyscopeApproximatedMesh->setSurfaceColor({0.9, 0.9, 0.9});

    // Draw wires
    polyscope::SurfaceGraphQuantity* showWires_approximated = polyscopeApproximatedMesh->addSurfaceGraphQuantity("wires", wirePos,edgeInds);
    showWires_approximated->setEnabled(true);
    showWires_approximated->setColor({0.0, 0.0, 0.0});


    approximated = true;
    // Save Mesh
    save_approximated_mesh();
    current_mode = APPROXIMATED;
}

void NebutaManager::save_approximated_mesh(){
    if(!approximated){
        developable_approximation();
    }

    std::string filename = pathM + "approximated_mesh.obj";
    igl::writeOBJ(filename, approximatedV, approximatedF);
}

void NebutaManager::prepare_tracing() {
    if (!remeshed){
        remesh_and_field_computation();
    }

    // Convert to TraceMesh
    std::vector<std::vector<int>> vec_OF;
    std::vector<std::vector<double>> vec_OV;
    for(int i=0; i<remeshedF.rows(); i++){
        vec_OF.push_back(std::vector<int>{remeshedF(i, 0), remeshedF(i, 1), remeshedF(i, 2)});
    }
    for(int i=0; i<remeshedV.rows(); i++){
        vec_OV.push_back(std::vector<double>{remeshedV(i, 0), remeshedV(i, 1), remeshedV(i, 2)});
    }
    vcg::tri::BuildMeshFromCoordVectorIndexVector(traceMesh, vec_OV, vec_OF);
    traceMesh.UpdateAttributes();

    // Copy the field
    auto loaded = traceMesh.LoadField(pathM + "_field.rosy");
    if(!loaded) {
        std::cerr << "Error when loading field" << std::endl;
        exit(1);
    }

    // Create the graph
    VGraph = new VertexFieldGraph<TraceMesh>(traceMesh);
    // Create the tracer
    PTr = new TracerType(*VGraph);

    init_structures();
}

void NebutaManager::init_structures() {
    PreProcessMesh(traceMesh);
    VGraph->InitGraph(false);
    PTr->InitTracer(Drift,false);
}

void NebutaManager::color_selected_patch(Eigen::MatrixXd &per_face_color, int patch_idx) {
    per_face_color = Eigen::MatrixXd::Zero(approximatedF.rows(), 3);

    for (int i=0; i<approximatedF.rows(); i++) {
        int max_limit = patch_to_max_face_idx[patch_idx];
        int min_limit = (patch_idx <= 0) ? 0 : patch_to_max_face_idx[patch_idx-1];
        if (i >= min_limit && i < max_limit) {
            per_face_color.row(i) = Eigen::RowVector3d(0.9, 0., 0.);
        }
        else {
            per_face_color.row(i) = Eigen::RowVector3d(0.9, 0.9, 0.9);
        }
    }
}

void NebutaManager::update_visualization() {
    switch (current_mode) {
        case ORIGINAL:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(true);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            if (polyscopeWireMesh != nullptr) polyscopeWireMesh->setEnabled(false);
            break;
        case REMESHED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(true);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            if (polyscopeWireMesh != nullptr) polyscopeWireMesh->setEnabled(false);
            break;
        case PARTITIONED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(true);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            if (polyscopeWireMesh != nullptr) polyscopeWireMesh->setEnabled(false);
            break;
        case APPROXIMATED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) {
                polyscopeApproximatedMesh->setEnabled(true);
            }
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            if (polyscopeWireMesh != nullptr) polyscopeWireMesh->setEnabled(false);
            break;

        case FABRICATIONMODE:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) {
                polyscopeApproximatedMesh->setEnabled(true);
                Eigen::MatrixXd per_face_color;
                color_selected_patch(per_face_color, fabrication_mode_patch_id);
                auto colq = polyscopeApproximatedMesh->addFaceColorQuantity("patch coloring", per_face_color);
                colq->setEnabled(true);
            }
            if (polyscopeWireMesh != nullptr) polyscopeWireMesh->setEnabled(false);
            break;

        case WIREONLY:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            if (polyscopeWireMesh != nullptr) polyscopeWireMesh->setEnabled(true);
            break;


            /* Show flattened mesh
            Eigen::MatrixXd flattenedV_display(flattenedVs[fabrication_mode_patch_id].rows(), 3);
            for(int i=0; i<flattenedV_display.rows(); i++){
                flattenedV_display(i, 0) = flattenedVs[fabrication_mode_patch_id](i, 0) / height_in_cm + 30;
                flattenedV_display(i, 2) = flattenedVs[fabrication_mode_patch_id](i, 1) / height_in_cm + 30;
                flattenedV_display(i, 1) = 0;
            }
            assert(flattenedV_display.rows() == flattenedFs[fabrication_mode_patch_id].maxCoeff() + 1);
            polyscopeFlattenedMesh = polyscope::registerSurfaceMesh("Flattened Mesh", flattenedV_display, flattenedFs[fabrication_mode_patch_id]);
            polyscopeFlattenedMesh->setEdgeWidth(1.);
            polyscopeFlattenedMesh->setEnabled(true);
            polyscopePatchMesh = polyscope::registerSurfaceMesh("Patch Mesh", approximatedVs[fabrication_mode_patch_id], flattenedFs[fabrication_mode_patch_id]);
            polyscopePatchMesh->setEdgeWidth(1.);
            polyscopePatchMesh->setEnabled(true);*/
            break;
    }
}

void NebutaManager::save_wire_indices() {
    igl::writeOBJ(pathM + "_remeshed.obj", remeshedV, remeshedF);
    std::ofstream ofs(pathM + "_wire_indices.txt");

    std::vector<std::vector<size_t>> CurrV;
    std::vector<std::vector<size_t>> _CurrDir;
    std::vector<bool> IsLoop;

    PTr->GetCurrVertDir(CurrV, _CurrDir, IsLoop);

    int index_size = 0;

    for (auto i = 0; i < CurrV.size(); i++) {
        index_size += CurrV[i].size();
    }

    ofs << index_size << std::endl;

    for (auto i = 0; i < CurrV.size(); i++) {
        for (auto j = 0; j < CurrV[i].size(); j++) {
            ofs << CurrV[i][j] << " ";
        }
        ofs << std::endl;
    }
    ofs.close();
}

void NebutaManager::save_wire_positions() {
    std::vector<std::vector<size_t>> CurrV;
    std::vector<std::vector<size_t>> _CurrDir;
    std::vector<bool> IsLoop;

    PTr->GetCurrVertDir(CurrV, _CurrDir, IsLoop);

    std::ofstream fw(pathM + "_wire_positions.txt");
    for(int i=0; i<CurrV.size(); i++){
        fw << "|";
        if(IsLoop[i]){
            fw << "L|\n";
        }else{
            fw << "P|\n";
        }
        for(int j=0; j<CurrV[i].size(); j++){
            size_t vi = CurrV[i][j];
            fw << partitionedV(vi, 0) << " " << partitionedV(vi, 1) << " " << partitionedV(vi, 2) << "\n";
        }
        fw << std::flush;
    }
    fw.close();
}

void NebutaManager::save_2d_pattern() {
    if(!approximated){
        developable_approximation();
    }

    int svg_fileid = 1;
    std::string svg_path = pathM + "_2d_pattern_1.svg";

    svg_writer* now_svg = new svg_writer(svg_path, 21.0, 29.7);

    for (int i=0; i<flattenedVs.size(); i++){
        bool needs_new_svg = !now_svg->add_patch(flattenedVs[i], flattenedFs[i], i);
        if(needs_new_svg){
            delete now_svg;
            svg_fileid++;
            svg_path = pathM + "_2d_pattern_" + std::to_string(svg_fileid) + ".svg";
            now_svg = new svg_writer(svg_path, 21.0, 29.7);
            std::cerr << "New svg created" << std::endl;
            if(!now_svg->add_patch(flattenedVs[i], flattenedFs[i], i)){
                std::cerr << "Error when writing svg" << std::endl;
                delete now_svg;
                break;
            }
        }
    }
    now_svg->finish();
    delete now_svg;
}


double NebutaManager::get_wire_length() {
    if(!partitioned){
        return 0;
    }
    double wire_length = 0;

    std::vector<std::vector<size_t>> CurrV;
    std::vector<std::vector<size_t>> _CurrDir;
    std::vector<bool> IsLoop;

    PTr->GetCurrVertDir(CurrV, _CurrDir, IsLoop);

    for (auto i = 0; i < CurrV.size(); i++) {
        int limit = (IsLoop[i]) ? CurrV[i].size() : CurrV[i].size() - 1;
        for (auto j = 0; j < limit; j++) {
            wire_length += (partitionedV.row(CurrV[i][j]) - partitionedV.row(CurrV[i][(j + 1) % CurrV[i].size()])).norm();
        }
    }

    return wire_length;
}
int NebutaManager::get_patch_cnt() {
    if (!partitioned) {
        return 0;
    }
    return PTr->Partitions.size();
}
int NebutaManager::get_wire_cnt() {
    if (!partitioned) {
        return 0;
    }

    std::vector<std::vector<size_t>> CurrV;
    std::vector<std::vector<size_t>> _CurrDir;
    std::vector<bool> IsLoop;

    PTr->GetCurrVertDir(CurrV, _CurrDir, IsLoop);

    return CurrV.size();
}
double NebutaManager::get_hausdorff_distance(){
    if(!approximated){
        return -1;
    }
    double hd;
    igl::hausdorff(approximatedV, approximatedF, originalV, originalF, hd);
    return hd;
}

int NebutaManager::get_patch_quality_mode() {
    return (int)quality_measure;
}

#endif //LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H
