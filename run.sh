#!/bin/bash
# Add the directory containing libpddlboat-cpp.dylib to DYLD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$HOME/Desktop/Classes/AlgoRobotics/Research/Code/pddlboat/build/release

# Add Miniconda environment's lib directory to DYLD_LIBRARY_PATH
export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$HOME/miniconda3/envs/spotenv/lib

# Execute the main program with all command-line arguments passed to the script
./main "$@"

