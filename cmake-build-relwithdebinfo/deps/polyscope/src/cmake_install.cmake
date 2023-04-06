# Install script for directory: /Users/nagata/git/nebuta-design-updated/deps/polyscope/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "RelWithDebInfo")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/objdump")
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/lib/libpolyscope.a")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpolyscope.a" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpolyscope.a")
    execute_process(COMMAND "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/ranlib" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libpolyscope.a")
  endif()
endif()

if(CMAKE_INSTALL_COMPONENT STREQUAL "Unspecified" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/include/polyscope/affine_remapper.h;/usr/local/include/polyscope/affine_remapper.ipp;/usr/local/include/polyscope/camera_parameters.h;/usr/local/include/polyscope/color_management.h;/usr/local/include/polyscope/colors.h;/usr/local/include/polyscope/combining_hash_functions.h;/usr/local/include/polyscope/curve_network.h;/usr/local/include/polyscope/curve_network.ipp;/usr/local/include/polyscope/curve_network_color_quantity.h;/usr/local/include/polyscope/curve_network_quantity.h;/usr/local/include/polyscope/curve_network_scalar_quantity.h;/usr/local/include/polyscope/curve_network_vector_quantity.h;/usr/local/include/polyscope/disjoint_sets.h;/usr/local/include/polyscope/file_helpers.h;/usr/local/include/polyscope/histogram.h;/usr/local/include/polyscope/image_scalar_artist.h;/usr/local/include/polyscope/imgui_config.h;/usr/local/include/polyscope/messages.h;/usr/local/include/polyscope/options.h;/usr/local/include/polyscope/persistent_value.h;/usr/local/include/polyscope/pick.h;/usr/local/include/polyscope/pick.ipp;/usr/local/include/polyscope/point_cloud.h;/usr/local/include/polyscope/point_cloud.ipp;/usr/local/include/polyscope/point_cloud_color_quantity.h;/usr/local/include/polyscope/point_cloud_quantity.h;/usr/local/include/polyscope/point_cloud_scalar_quantity.h;/usr/local/include/polyscope/point_cloud_parameterization_quantity.h;/usr/local/include/polyscope/point_cloud_vector_quantity.h;/usr/local/include/polyscope/polyscope.h;/usr/local/include/polyscope/quantity.h;/usr/local/include/polyscope/quantity.ipp;/usr/local/include/polyscope/color_maps.h;/usr/local/include/polyscope/engine.h;/usr/local/include/polyscope/engine.ipp;/usr/local/include/polyscope/ground_plane.h;/usr/local/include/polyscope/material_defs.h;/usr/local/include/polyscope/materials.h;/usr/local/include/polyscope/ribbon_artist.h;/usr/local/include/polyscope/scaled_value.h;/usr/local/include/polyscope/screenshot.h;/usr/local/include/polyscope/slice_plane.h;/usr/local/include/polyscope/standardize_data_array.h;/usr/local/include/polyscope/structure.h;/usr/local/include/polyscope/structure.ipp;/usr/local/include/polyscope/surface_color_quantity.h;/usr/local/include/polyscope/surface_count_quantity.h;/usr/local/include/polyscope/surface_distance_quantity.h;/usr/local/include/polyscope/surface_graph_quantity.h;/usr/local/include/polyscope/surface_input_curve_quantity.h;/usr/local/include/polyscope/surface_mesh.h;/usr/local/include/polyscope/surface_mesh.ipp;/usr/local/include/polyscope/surface_mesh_io.h;/usr/local/include/polyscope/surface_mesh_quantity.h;/usr/local/include/polyscope/surface_parameterization_enums.h;/usr/local/include/polyscope/surface_parameterization_quantity.h;/usr/local/include/polyscope/surface_scalar_quantity.h;/usr/local/include/polyscope/surface_selection_quantity.h;/usr/local/include/polyscope/surface_subset_quantity.h;/usr/local/include/polyscope/surface_vector_quantity.h;/usr/local/include/polyscope/trace_vector_field.h;/usr/local/include/polyscope/types.h;/usr/local/include/polyscope/utilities.h;/usr/local/include/polyscope/view.h;/usr/local/include/polyscope/volume_mesh.h;/usr/local/include/polyscope/volume_mesh.ipp;/usr/local/include/polyscope/volume_mesh_quantity.h")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  file(INSTALL DESTINATION "/usr/local/include/polyscope" TYPE FILE FILES
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//affine_remapper.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//affine_remapper.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//camera_parameters.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//color_management.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//colors.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//combining_hash_functions.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//curve_network.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//curve_network.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//curve_network_color_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//curve_network_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//curve_network_scalar_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//curve_network_vector_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//disjoint_sets.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//file_helpers.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//histogram.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//image_scalar_artist.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//imgui_config.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//messages.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//options.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//persistent_value.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//pick.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//pick.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//point_cloud.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//point_cloud.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//point_cloud_color_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//point_cloud_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//point_cloud_scalar_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//point_cloud_parameterization_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//point_cloud_vector_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//polyscope.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//quantity.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//render/color_maps.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//render/engine.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//render/engine.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//render/ground_plane.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//render/material_defs.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//render/materials.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//ribbon_artist.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//scaled_value.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//screenshot.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//slice_plane.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//standardize_data_array.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//structure.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//structure.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_color_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_count_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_distance_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_graph_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_input_curve_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_mesh.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_mesh.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_mesh_io.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_mesh_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_parameterization_enums.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_parameterization_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_scalar_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_selection_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_subset_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//surface_vector_quantity.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//trace_vector_field.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//types.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//utilities.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//view.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//volume_mesh.h"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//volume_mesh.ipp"
    "/Users/nagata/git/nebuta-design-updated/deps/polyscope/src/../include/polyscope//volume_mesh_quantity.h"
    )
endif()

