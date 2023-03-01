//
// Created by Naoki Agata on 2023/02/23.
//

#ifndef LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H
#define LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H

#include <Eigen/Core>
#include <string>
#include "mesh_manager.h"
#include "fields/field_smoother.h"
#include "triangle_mesh_type.h"
#include "poly_mesh_type.h"
#include "AutoRemesher.h"
#include "mesh_field_smoother.h"
#include "tracing/mesh_type.h"
#include "tracing/patch_tracer.h"
#include "mesh_quality.h"
#include "igl/read_triangle_mesh.h"
#include "vcg/complex/algorithms/create/platonic.h"
#include "cstdio"
#include "tracing/tracer_interface.h"
#include "triangle_strips.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using TracerType = PatchTracer<TraceMesh, MeshQuality<TraceMesh>>;
using FieldSmootherType = vcg::tri::FieldSmoother<FieldTriMesh>;

class NebutaManager {
public:
    enum display_mode {ORIGINAL, REMESHED, PARTITIONED, APPROXIMATED};
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

    display_mode current_mode = ORIGINAL;
    ParamMode quality_measure = PMHausdorf;

    bool remeshed = false;
    bool partitioned = false;
    bool approximated = false;

    void set_parameters();

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
    double sample_ratio = 0.1;
    bool split_on_removal = false;
    bool CClarkability = false;
    bool match_valence = false;
    bool check_quality_functor = true;
    bool add_only_needed = true;
    bool final_removal = true;
    bool meta_mesh_collapse = false;
    bool force_split = false;


    NebutaManager();
    ~NebutaManager();
    void load_mesh(std::string mesh_filename);
    void remesh_and_field_computation();
    void init_structures();
    void prepare_tracing();
    void patch_tracing();
    void developable_approximation();

    void update_visualization();
};

//
// Created by Naoki Agata on 2023/02/23.
//

NebutaManager::NebutaManager()
        : VGraph(nullptr), PTr(nullptr)
{

}

NebutaManager::~NebutaManager() {
    if(VGraph != nullptr) delete VGraph;
    if(PTr != nullptr) delete PTr;
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

    // Convert to VCG mesh
    ConvertToVCGMesh(originalVCGMesh, originalV, originalF);
    originalVCGMesh.UpdateDataStructures();

    // Get the path of the mesh
    auto dot_pos = filename.find_first_of('.');
    pathM = filename.substr(0, dot_pos);

    // Register to polyscope
    polyscopeOriginalMesh = polyscope::registerSurfaceMesh("Original Mesh", originalV, originalF);
    polyscopeOriginalMesh->setSurfaceColor({0.9, 0.9, 0.9});
    current_mode = ORIGINAL;
}

void NebutaManager::remesh_and_field_computation() {
    if(remeshed){
        current_mode = REMESHED;
        return;
    }

    typename MeshPrepocess<FieldTriMesh>::BatchParam BPar;
    BPar.DoRemesh=true;
    BPar.UpdateSharp= false;
    MeshPrepocess<FieldTriMesh>::BatchProcess(originalVCGMesh, BPar, FieldParam);

    // Save the temporal RoSy Field to file
    originalVCGMesh.SaveField(pathM + "_field.rosy");

    // Store remeshed mesh in igl format
    vcg::tri::MeshToMatrix<FieldTriMesh>::GetTriMeshData(originalVCGMesh, remeshedF, remeshedV);

    // Register to polyscope
    polyscopeRemeshedMesh = polyscope::registerSurfaceMesh("Remeshed Mesh", remeshedV, remeshedF);
    polyscopeRemeshedMesh->setSurfaceColor({0.9, 0.9, 0.9});
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

    MeshQuality<TraceMesh>::UVMode() = quality_measure;
}

void NebutaManager::patch_tracing() {
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
    current_mode = PARTITIONED;

    // Draw wires
    std::vector<std::vector<size_t> > CurrV;
    std::vector<std::vector<size_t> > _CurrDir;
    std::vector<bool> IsLoop;

    PTr->GetCurrVertDir(CurrV, _CurrDir, IsLoop);

    std::vector<Eigen::Vector3d> wirePos;
    std::vector<std::array<size_t, 2>> edgeInds;

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

    polyscope::SurfaceGraphQuantity* showWires = polyscopePartitionedMesh->addSurfaceGraphQuantity("wires", wirePos,edgeInds);
    showWires->setEnabled(true);
    showWires->setColor({0.0, 0.0, 0.0});
    current_mode = PARTITIONED;
    partitioned = true;
}

void NebutaManager::developable_approximation() {
    if(!partitioned){
        prepare_tracing();
        patch_tracing();
    }

    TraceMesh approximatedMesh;

    for(auto i=0; i<PTr->Partitions.size(); i++){
        TraceMesh patchMesh;
        PTr->GetPatchMesh(i, patchMesh, false);
        Eigen::MatrixXd patchV, resultPatchV;
        Eigen::MatrixXi patchF, resultPatchF;
        vcg::tri::MeshToMatrix<TraceMesh>::GetTriMeshData(patchMesh, patchF, patchV);
        tri_strip::approximate_single_patch(patchV, patchF, resultPatchV, resultPatchF, tri_strip::per_face_distance);
        ConvertToVCGMesh(patchMesh, resultPatchV, resultPatchF);
        patchMesh.UpdateAttributes();

        vcg::tri::Append<TraceMesh,TraceMesh>::MeshAppendConst(approximatedMesh, patchMesh);
    }

    // Store approximated mesh in igl format
    vcg::tri::MeshToMatrix<TraceMesh>::GetTriMeshData(approximatedMesh, approximatedF, approximatedV);
    polyscopeApproximatedMesh = polyscope::registerSurfaceMesh("Approximated Mesh", approximatedV, approximatedF);
    polyscopeApproximatedMesh->setSurfaceColor({0.9, 0.9, 0.9});
    current_mode = APPROXIMATED;
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
    }else{
        std::remove((pathM + "_field.rosy").c_str());
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

void NebutaManager::update_visualization() {
    switch (current_mode) {
        case ORIGINAL:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(true);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            break;
        case REMESHED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(true);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            break;
        case PARTITIONED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(true);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            break;
        case APPROXIMATED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(true);
            break;
    }
}

#endif //LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H
