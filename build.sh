#!/usr/bin/env bash

# Use this to build Box3D on any system with a bash shell
rm -rf build

cmake -S . -B build
cmake --build build
