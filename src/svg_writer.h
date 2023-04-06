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

    double x;
    double y;

    double curr_max_y;

    std::string svg_header;
    std::string svg_body;
    std::string svg_footer;

    void new_line(){
        x = 0;
        y = curr_max_y;
    }

    void finish(){
        std::ofstream svg_file;
        svg_file.open(svg_file_name);
        svg_file << svg_header;
        svg_file << svg_body;
        svg_file << svg_footer;
        svg_file.close();
    }

public:
    svg_writer(std::string file_name, double width, double height){
        svg_file_name = file_name;
        max_width = width;
        max_height = height;
        x = 0;
        y = 0;

        svg_header = "<?xml version=\"1.0\" encoding=\"UTF-8\" standalone=\"no\"?>\n"
                     "<svg xmlns=\"http://www.w3.org/2000/svg\" xmlns:svg=\"http://www.w3.org/2000/svg\" version=\"1.1\" width=\""
                     + std::to_string(max_width) + "\" height=\"" + std::to_string(max_height) + /*"\" viewbox=\"0 0 " + std::to_string(max_width) + " " + std::to_string(max_height) +*/ "\">\n";
        svg_body = "";
        svg_footer = "</svg>\n";
    }

    bool add_patch(Eigen::MatrixXd &V, Eigen::MatrixXi &F, int patch_index){

        double bbheight = V.col(1).maxCoeff();
        double bbwidth = V.col(0).maxCoeff();

        std::cerr << "Patch: " << patch_index << " w: " << bbwidth << " h: " << bbheight << std::endl;

        if (x + bbwidth > max_width){
            new_line();
        }

        if (y + bbheight > max_height){
            finish();
            return false;
        }

        Eigen::VectorXi bnd;
        igl::boundary_loop(F, bnd);

        std::string svg_patch = "<path d=\"M";
        for (int i = 0; i < bnd.rows() + 1; i++){
            int idx = bnd(i % V.rows());
            svg_patch += std::to_string(x + V(idx, 0)) + "," + std::to_string(y +V(idx, 1)) + " ";
        }
        svg_patch += "Z\" stroke=\"black\" stroke-width=\"0.1\" fill=\"none\" />\n";
        svg_body += svg_patch;

        //std::string svg_id_text = "<text fontsize=\"0.3\" x=\"" + std::to_string(x + V.col(0).mean()) + "\" y=\"" + std::to_string(y + V.col(1).mean()) + "\" fill=\"black\">" + std::to_string(patch_index) + "</text>\n";
        //svg_body += svg_id_text;

        x += bbwidth;
        curr_max_y = std::max(curr_max_y, y + bbheight);

        return true;
    }
};


#endif //NEBUTA_DESIGNER_SVG_WRITER_H
