#include "polyscope/polyscope.h"
#include "polyscope/messages.h"
#include "polyscope/surface_mesh.h"

#include <iostream>
#include <utility>

#include "args/args.hxx"
#include "json/json.hpp"

#include "NebutaManager.h"

// The mesh, Eigen representation
Eigen::MatrixXd meshV;
Eigen::MatrixXi meshF;

NebutaManager nebutaManager;

void callback() {
    if (ImGui::Button("Show original mesh")) {
        nebutaManager.update_visualization();
    }
    ImGui::SameLine();

    if (ImGui::Button("Remesh")) {
        nebutaManager.remesh_and_field_computation();
        nebutaManager.update_visualization();
    }
    ImGui::SameLine();

    if (ImGui::Button("Partition")){
        nebutaManager.prepare_tracing();
        nebutaManager.patch_tracing();
        nebutaManager.update_visualization();
    }
    ImGui::SameLine();

    if (ImGui::Button("Approximate")){
        nebutaManager.developable_approximation();
        nebutaManager.update_visualization();
    }

    if (ImGui::Button("Save SVG")){
        nebutaManager.save_2d_pattern();
        nebutaManager.update_visualization();
    }

    ImGui::SameLine();
    if (ImGui::Button("Do Test")){
        nebutaManager.do_test();
        nebutaManager.update_visualization();
    }

    ImGui::Text("Mesh Result");
    static int display_mesh_mode = 0;
    enum {
        ORIGINAL,
        REMESHED,
        PARTITIONED,
        APPROXIMATED,
        FLATTENED,
        WIREONLY
    };
    if (ImGui::RadioButton("Original", &display_mesh_mode, ORIGINAL)) {
        nebutaManager.set_visualization_mode(NebutaManager::ORIGINAL);
        nebutaManager.update_visualization();
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Remeshed", &display_mesh_mode, REMESHED)) {
        nebutaManager.set_visualization_mode(NebutaManager::REMESHED);
        nebutaManager.update_visualization();
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Partitioned", &display_mesh_mode, PARTITIONED)) {
        nebutaManager.set_visualization_mode(NebutaManager::PARTITIONED);
        nebutaManager.update_visualization();
    }
    ImGui::SameLine();
    if (ImGui::RadioButton("Approximated", &display_mesh_mode, APPROXIMATED)) {
        nebutaManager.set_visualization_mode(NebutaManager::APPROXIMATED);
        nebutaManager.update_visualization();
    }
    if (ImGui::RadioButton("Fabrication Mode", &display_mesh_mode, FLATTENED)) {
        nebutaManager.set_visualization_mode(NebutaManager::FABRICATIONMODE);
        nebutaManager.update_visualization();
    }
    if (ImGui::RadioButton("Wire Only", &display_mesh_mode, WIREONLY)){
        nebutaManager.set_visualization_mode(NebutaManager::WIREONLY);
        nebutaManager.update_visualization();
    }
    if (display_mesh_mode == FLATTENED) {
        if(ImGui::InputInt("Patch ID", &nebutaManager.fabrication_mode_patch_id)){
            nebutaManager.update_visualization();
        };
    }

    if(ImGui::TreeNode("Patch Quality Measures")) {
        static int patch_quality_mode = nebutaManager.get_patch_quality_mode();
        enum {
            CONFORMAL,
            ARAP,
            GAUSSIANCURVATURE,
            GAUSSIMAGETHINNESS,
            HAUSDORFFDISTANCE
        };
        if (ImGui::RadioButton("Conformal", &patch_quality_mode, CONFORMAL)) {
            nebutaManager.set_patch_quality_mode(ParamMode::PMConformal);
        }
        if (ImGui::RadioButton("ARAP", &patch_quality_mode, ARAP)) {
            nebutaManager.set_patch_quality_mode(ParamMode::PMArap);
        }
        if (ImGui::RadioButton("Gaussian Curvature", &patch_quality_mode, GAUSSIANCURVATURE)) {
            nebutaManager.set_patch_quality_mode(ParamMode::PMGaussianCurvature);
        }
        if (ImGui::RadioButton("Gauss Image Thinness", &patch_quality_mode, GAUSSIMAGETHINNESS)) {
            nebutaManager.set_patch_quality_mode(ParamMode::PMGaussImageThinness);
        }
        if (ImGui::RadioButton("Hausdorff Distance", &patch_quality_mode, HAUSDORFFDISTANCE)) {
            nebutaManager.set_patch_quality_mode(ParamMode::PMHausdorff);
        }

        if (patch_quality_mode == CONFORMAL || patch_quality_mode == ARAP) {
            float minQ = MeshQuality<TraceMesh>::MinQ();
            ImGui::InputFloat("MinQ", &minQ);
            MeshQuality<TraceMesh>::MinQ() = minQ;
            float maxQ = MeshQuality<TraceMesh>::MaxQ();
            ImGui::InputFloat("MaxQ", &maxQ);
            MeshQuality<TraceMesh>::MaxQ() = maxQ;
        }
        if (patch_quality_mode == GAUSSIANCURVATURE) {
            float maxV = MeshQuality<TraceMesh>::MaxSumOfGaussianCurvature();
            ImGui::InputFloat("Threshold", &maxV);
            MeshQuality<TraceMesh>::MaxSumOfGaussianCurvature() = maxV;
        }
        if (patch_quality_mode == GAUSSIMAGETHINNESS) {
            float maxV = MeshQuality<TraceMesh>::MaxGaussImageThickness() * 100;
            ImGui::InputFloat("Threshold", &maxV);
            MeshQuality<TraceMesh>::MaxGaussImageThickness() = maxV / 100;
        }
        if (patch_quality_mode == HAUSDORFFDISTANCE) {
            float maxV = MeshQuality<TraceMesh>::MaxHausdorff();
            ImGui::InputFloat("Threshold", &maxV);
            MeshQuality<TraceMesh>::MaxHausdorff() = maxV;
        }
        ImGui::TreePop();
    }
    if(ImGui::TreeNode("Developable Approximation Method")) {
        static int approximation_mode = 1;
        enum {
            QSLIM,
            DP_PERFACE_DISTANCE
        };
        ImGui::RadioButton("Mitani and Suzuki, 2004", &approximation_mode, QSLIM);
        ImGui::RadioButton("DP Per-Edge Distance", &approximation_mode, DP_PERFACE_DISTANCE);

        switch (approximation_mode) {
            case QSLIM:
                approximate_single_patch::approx_mode = approximate_single_patch::QSlim;
                break;
            case DP_PERFACE_DISTANCE:
                approximate_single_patch::approx_mode = approximate_single_patch::DP_perface_distance;
                break;
        }
        ImGui::TreePop();
    }

    bool trace_only_loop = !nebutaManager.allow_trace_loopish;
    if(ImGui::Checkbox("Trace only loop", &trace_only_loop)) {
        nebutaManager.allow_trace_loopish = !trace_only_loop;
    }

    ImGui::Checkbox("Split on removal", &nebutaManager.split_on_removal);
    float sample_ratio_slider = nebutaManager.sample_ratio;
    ImGui::SliderFloat("Sample ratio", &sample_ratio_slider, 0.0f, 1.0f);
    nebutaManager.sample_ratio = sample_ratio_slider;

    ImGui::InputDouble("Height (cm)", &nebutaManager.height_in_cm);

    ImGui::Text("Patch count: %d", nebutaManager.get_patch_cnt());
    ImGui::Text("Wire count: %d", nebutaManager.get_wire_cnt());
    ImGui::Text("Wire length: %f", nebutaManager.get_wire_length());
    ImGui::Text("Approximation error: %f", nebutaManager.get_hausdorff_distance());
}

int main(int argc, char **argv) {
  // Configure the argument parser
  args::ArgumentParser parser("A simple demo of Polyscope with libIGL.\nBy "
                              "Nick Sharp (nsharp@cs.cmu.edu)",
                              "");
  args::Positional<std::string> inFile(parser, "mesh", "input mesh");

  // Parse args
  try {
    parser.ParseCLI(argc, argv);
  } catch (args::Help) {
    std::cout << parser;
    return 0;
  } catch (args::ParseError e) {
    std::cerr << e.what() << std::endl;

    std::cerr << parser;
    return 1;
  }

  // Options
  polyscope::options::autocenterStructures = true;
  polyscope::view::windowWidth = 1024;
  polyscope::view::windowHeight = 1024;

  // Initialize polyscope
  polyscope::init();

  // Load the mesh
  std::string filename = args::get(inFile);
  std::cout << "loading: " << filename << std::endl;
  nebutaManager.load_mesh(filename);

  // Add the callback
  polyscope::state::userCallback = callback;

  // Show the gui
  polyscope::show();

  return 0;
}
