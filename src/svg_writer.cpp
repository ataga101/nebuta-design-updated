//
// Created by Naoki Agata on 2023/04/07.
//

#include "svg_writer.h"


void svg_writer::new_line() {
    x = fontsize;
    y = curr_max_y;
}

void svg_writer::finish() {
    std::ofstream svg_file;
    svg_file.open(svg_file_name);
    svg_file << svg_header;
    svg_file << svg_body;
    svg_file << svg_footer;
    svg_file.close();
}

svg_writer::svg_writer(std::string file_name, double width, double height) {
    svg_scale = 30 * 1.259862560447951;
    fontsize = 0.1;
    svg_file_name = file_name;
    max_width = width;
    max_height = height;
    x = fontsize * 3;
    y = fontsize * 4;

    svg_header = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n"
                 "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:svg=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\""
                 + std::to_string(max_width * svg_scale) + "\" height=\"" + std::to_string(max_height * svg_scale) +
                 "\">\n";
    svg_body = "";
    svg_footer = "</svg>\n";
}

bool svg_writer::add_patch(Eigen::MatrixXd &V, Eigen::MatrixXi &F, int patch_index) {
    double bbheight = V.col(1).maxCoeff();
    double bbwidth = V.col(0).maxCoeff();
    double offset = fontsize * 3;

    std::cerr << "Patch: " << patch_index << " w: " << bbwidth << " h: " << bbheight << std::endl;

    if (x + bbwidth + offset > max_width) {
        new_line();
    }

    if (y + bbheight + 2 * offset > max_height) {
        finish();
        return false;
    }

    Eigen::VectorXi bnd;
    igl::boundary_loop(F, bnd);

    std::string svg_patch = "<path d=\"M ";
    for (int i = 0; i < bnd.rows() + 1; i++) {
        int idx = bnd(i % V.rows());
        svg_patch += " " + std::to_string((x + V(idx, 0) + offset) * svg_scale) + "," + std::to_string((y + V(idx, 1) + offset) * svg_scale) + " ";
    }
    svg_patch += "Z\" stroke=\"black\" stroke-width=\"" + std::to_string(0.1 * svg_scale) + "\" fill=\"none\" />\n";
    svg_body += svg_patch;

    std::string svg_id_text = "<text fontsize=\"" + std::to_string(fontsize * svg_scale) + "\" x=\"" + std::to_string(x * svg_scale) + "\" y=\"" + std::to_string(y * svg_scale) + "\" fill=\"black\">" + std::to_string(patch_index) + "</text>\n";
    svg_body += svg_id_text;

    x += bbwidth + offset;
    curr_max_y = std::max(curr_max_y, y + bbheight + 2 * offset);

    return true;
}