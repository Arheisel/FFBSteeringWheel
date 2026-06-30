#!/bin/bash
# Compile the firmware using all available CPU cores
if [ "$1" == "-c" ]; then
    echo "Cleaning build directory..."
    make clean
fi

echo "Compiling firmware..."
make -j$(nproc)
