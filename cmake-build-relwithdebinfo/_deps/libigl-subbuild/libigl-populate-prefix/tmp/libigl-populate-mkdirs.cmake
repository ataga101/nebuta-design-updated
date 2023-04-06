# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-src"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-build"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-subbuild/libigl-populate-prefix"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-subbuild/libigl-populate-prefix/tmp"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-subbuild/libigl-populate-prefix/src/libigl-populate-stamp"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-subbuild/libigl-populate-prefix/src"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-subbuild/libigl-populate-prefix/src/libigl-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-subbuild/libigl-populate-prefix/src/libigl-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/libigl-subbuild/libigl-populate-prefix/src/libigl-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
