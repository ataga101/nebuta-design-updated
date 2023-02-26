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

// Options for algorithms
int iVertexSource = 7;

NebutaManager nebutaManager;

void callback() {
    if (ImGui::Button("Show original mesh")) {
        nebutaManager.update_visualization();
    }

    if (ImGui::Button("Remesh")) {
        nebutaManager.remesh_and_field_computation();
        nebutaManager.update_visualization();
    }

    if (ImGui::Button("Partition")){
        nebutaManager.prepare_tracing();
        nebutaManager.patch_tracing();
        nebutaManager.update_visualization();
    }

    if (ImGui::Button("Approximate")){
        nebutaManager.developable_approximation();
        nebutaManager.update_visualization();
    }
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
