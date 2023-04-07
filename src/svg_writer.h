//
// Created by Naoki Agata on 2023/04/05.
//
#include "string"
#include "vector"
#include "fstream"
#include "Eigen/Core"
#include "igl/boundary_loop.h"

#ifndef NEBUTA_DESIGNER_SVG_WRITER_H
#define NEBUTA_DESIGNER_SVG_WRITER_H


class svg_writer {
private:
    std::string svg_file_name;

    double max_width;
    double max_height;

    double svg_scale;
    double fontsize;

    double x;
    double y;

    double curr_max_y;

    std::string svg_header;
    std::string svg_body;
    std::string svg_footer;

    void new_line();


public:
    svg_writer(std::string file_name, double width, double height);

    void finish();
    bool add_patch(Eigen::MatrixXd &V, Eigen::MatrixXi &F, int patch_index);
};


#endif //NEBUTA_DESIGNER_SVG_WRITER_H
