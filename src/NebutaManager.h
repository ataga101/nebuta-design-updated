//
// Created by Naoki Agata on 2023/02/23.
//

#ifndef LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H
#define LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H

#include <Eigen/Core>
#include <string>
#include "mesh_manager.h"
#include "svg_writer.h"
#include "flatten_triangle_strip.h"
#include "fields/field_smoother.h"
#include "triangle_mesh_type.h"
#include "poly_mesh_type.h"
#include "AutoRemesher.h"
#include "mesh_field_smoother.h"
#include "tracing/mesh_type.h"
#include "tracing/patch_tracer.h"
#include "mesh_quality.h"
#include "igl/read_triangle_mesh.h"
#include "igl/harmonic.h"
#include "igl/map_vertices_to_circle.h"
#include "vcg/complex/algorithms/create/platonic.h"
#include "cstdio"
#include "tracing/tracer_interface.h"
#include "approximate_single_patch.h"
#include "developableflow/timestep.h"

#include "polyscope/polyscope.h"
#include "polyscope/surface_mesh.h"

using TracerType = PatchTracer<TraceMesh, MeshQuality<TraceMesh>>;
using FieldSmootherType = vcg::tri::FieldSmoother<FieldTriMesh>;

class NebutaManager {
public:
    enum display_mode {ORIGINAL, REMESHED, PARTITIONED, APPROXIMATED, FLATTENED};
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

    display_mode current_mode = ORIGINAL;
    ParamMode quality_measure = PMHausdorff;

    bool remeshed = false;
    bool partitioned = false;
    bool approximated = false;

    // Wire graph infos
    std::vector<Eigen::Vector3d> wirePos;
    std::vector<std::array<size_t, 2>> edgeInds;

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
    double sample_ratio = 1;
    double height_in_cm = 15;
    bool split_on_removal = false;
    bool CClarkability = false;
    bool match_valence = false;
    bool check_quality_functor = true;
    bool add_only_needed = true;
    bool final_removal = true;
    bool meta_mesh_collapse = false;
    bool force_split = false;
    bool allow_trace_loopish = false;

    int fabrication_mode_patch_id;

    std::vector<Eigen::MatrixXd> flattenedVs;
    std::vector<Eigen::MatrixXi> flattenedFs;
    std::vector<Eigen::MatrixXd> approximatedVs;

    NebutaManager();
    ~NebutaManager();
    void load_mesh(std::string mesh_filename);
    void remesh_and_field_computation();
    void init_structures();
    void prepare_tracing();
    void patch_tracing();
    void developable_approximation();

    int get_num_patches();

    void set_visualization_mode(display_mode mode);
    void set_patch_quality_mode(ParamMode mode);
    void update_visualization();

    void save_wire_indices();
    void save_2d_pattern();
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
    if(VGraph != nullptr) delete VGraph;
    if(PTr != nullptr) delete PTr;
    std::remove((pathM + "_field.rosy").c_str());
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
    double max_height = (originalV.colwise().maxCoeff() - originalV.colwise().minCoeff()).maxCoeff();
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
    current_mode = PARTITIONED;
    partitioned = true;
}

void NebutaManager::developable_approximation() {
    if(!partitioned){
        prepare_tracing();
        patch_tracing();
    }

    TraceMesh approximatedMesh;

    flattenedVs.clear();
    flattenedFs.clear();
    approximatedVs.clear();

    for(auto i=0; i<PTr->Partitions.size(); i++){
        TraceMesh patchMesh;
        PTr->GetPatchMesh(i, patchMesh, false);
        Eigen::MatrixXd patchV, resultPatchV;
        Eigen::MatrixXi patchF, resultPatchF;
        vcg::tri::MeshToMatrix<TraceMesh>::GetTriMeshData(patchMesh, patchF, patchV);

        approximate_single_patch::approximate_single_patch(patchV, patchF, resultPatchV, resultPatchF);

        approximatedVs.push_back(resultPatchV);

        // Flatten the mesh
        Eigen::MatrixXd flattenedPatchV;

        Eigen::VectorXi bnd;
        igl::boundary_loop(resultPatchF, bnd);

        Eigen::MatrixXd scaledV = resultPatchV * height_in_cm;
        flatten_triangle_strip(scaledV, resultPatchF, Eigen::Vector3d::Zero(), flattenedPatchV);

        auto min_x = flattenedPatchV.col(0).minCoeff();
        auto min_y = flattenedPatchV.col(1).minCoeff();

        for (int i=0; i<flattenedPatchV.rows(); i++) {
            flattenedPatchV(i, 0) -= min_x;
            flattenedPatchV(i, 1) -= min_y;
        }

        flattenedVs.push_back(flattenedPatchV);
        flattenedFs.push_back(resultPatchF);

        // Update vcg mesh
        ConvertToVCGMesh(patchMesh, resultPatchV, resultPatchF);
        patchMesh.UpdateAttributes();

        vcg::tri::Append<TraceMesh,TraceMesh>::MeshAppendConst(approximatedMesh, patchMesh);
    }

    // Store approximated mesh in igl format
    vcg::tri::MeshToMatrix<TraceMesh>::GetTriMeshData(approximatedMesh, approximatedF, approximatedV);
    polyscopeApproximatedMesh = polyscope::registerSurfaceMesh("Approximated Mesh", approximatedV, approximatedF);
    polyscopeApproximatedMesh->setEdgeWidth(1.);
    polyscopeApproximatedMesh->setSurfaceColor({0.9, 0.9, 0.9});

    // Draw wires
    polyscope::SurfaceGraphQuantity* showWires_approximated = polyscopeApproximatedMesh->addSurfaceGraphQuantity("wires", wirePos,edgeInds);
    showWires_approximated->setEnabled(true);
    showWires_approximated->setColor({0.0, 0.0, 0.0});

    approximated = true;
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
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            break;
        case REMESHED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(true);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            break;
        case PARTITIONED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(true);
            if (polyscopeApproximatedMesh != nullptr) polyscopeApproximatedMesh->setEnabled(false);
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            break;
        case APPROXIMATED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr) {
                polyscopeApproximatedMesh->setEnabled(true);
                polyscopeApproximatedMesh->setTransparency(1);
            }
            if (polyscopeFlattenedMesh != nullptr) polyscopeFlattenedMesh->setEnabled(false);
            if (polyscopePatchMesh != nullptr) polyscopePatchMesh->setEnabled(false);
            break;

        case FLATTENED:
            if (polyscopeOriginalMesh != nullptr) polyscopeOriginalMesh->setEnabled(false);
            if (polyscopeRemeshedMesh != nullptr) polyscopeRemeshedMesh->setEnabled(false);
            if (polyscopePartitionedMesh != nullptr) polyscopePartitionedMesh->setEnabled(false);
            if (polyscopeApproximatedMesh != nullptr)
                polyscopeApproximatedMesh->setEnabled(false);


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
            polyscopePatchMesh->setEnabled(true);
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

int NebutaManager::get_num_patches(){
    return flattenedVs.size();
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
            std::cerr << "New patch created" << std::endl;
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

#endif //LIBIGL_POLYSCOPE_EXAMPLE_PROJECT_NEBUTAMANAGER_H
