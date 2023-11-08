# Computational Design of Nebuta-like Paper-on-Wire Artworks

This repository contains the code for the project ["Computational Design of Nebuta-like Paper-on-Wire Artworks"](https://dl.acm.org/doi/abs/10.1145/3588028.3603655) (SIGGRAPH 2023 Posters)
## To download and build

```
git clone --recurse-submodules https://github.com/ataga101/nebuta-design-updated.git
cd 
mkdir build
cd build
cmake ..
make -j4

./bin/nebuta-designer ../bunny.obj
```

## Usage

After the demo is launched, click the `"Save SVG"` button to segment mesh and following files are generated.
- **SVG file** (`*_2d_pattern*.svg`). Contains the 2D pattern of each paper patch.
- **Wire Position file** (`*_wire_positions.txt`). Contains the vertices positions of the polyline wire. Segments start with `|L|` means the path is a loop, and `|P|` means the path is not a loop.


## Note

This project is built on [libigl](https://libigl.github.io/), [polyscope](https://polyscope.run/), [xfield-tracer](https://github.com/nicopietroni/xfield_tracer.git) and [parafashion](https://github.com/nicopietroni/parafashion.git)

## License
GPLv3

## Citation
```
@inproceedings{10.1145/3588028.3603655,
author = {Agata, Naoki and Qi, Anran and Noma, Yuta and Shen, I-Chao and Igarashi, Takeo},
title = {Computational Design of Nebuta-like Paper-on-Wire Artworks},
year = {2023},
isbn = {9798400701528},
publisher = {Association for Computing Machinery},
address = {New York, NY, USA},
url = {https://doi.org/10.1145/3588028.3603655},
doi = {10.1145/3588028.3603655},
booktitle = {ACM SIGGRAPH 2023 Posters},
articleno = {7},
numpages = {2},
location = {Los Angeles, CA, USA},
series = {SIGGRAPH '23}
}
```