# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-src"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-build"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-subbuild/comiso-populate-prefix"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-subbuild/comiso-populate-prefix/tmp"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-subbuild/comiso-populate-prefix/src/comiso-populate-stamp"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-subbuild/comiso-populate-prefix/src"
  "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-subbuild/comiso-populate-prefix/src/comiso-populate-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-subbuild/comiso-populate-prefix/src/comiso-populate-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/nagata/git/nebuta-design-updated/cmake-build-relwithdebinfo/_deps/comiso-subbuild/comiso-populate-prefix/src/comiso-populate-stamp${cfgdir}") # cfgdir has leading slash
endif()
