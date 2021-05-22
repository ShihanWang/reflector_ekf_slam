#!/bin/sh

set -o errexit
set -o verbose

VERSION="1.13.0"

# Build and install Ceres.
git clone https://ceres-solver.googlesource.com/ceres-solver
cd ceres-solver
git checkout tags/${VERSION}
mkdir build
cd build
cmake .. -DCXX11=ON
make -j
sudo make install
