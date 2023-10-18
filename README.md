# GridWorld Planning with LTLf Formulas
This repository contains a C++ implementation of a planner that solves tasks in a grid-world environment using Linear Temporal Logic formulas (LTLf). The code represents grid-world domains, obstacles, states, and goals, and uses LTLf formulas to define temporal tasks.
## Installation
1. Ensure you have a C++17-compatible compiler installed.
2. Installing Required Libraries:
   * **Spot Library**: Install the Spot library using the following command:
     ```
     conda create --name spotenv python=3.8 # adjust as desired
     conda activate spotenv
     conda install -c conda-forge spot
     ```
     For detailed installation instructions, [click here](https://spot.lre.epita.fr/install.html).
   * **Graphviz**: This is required for automaton visualization. You can run the following (if you use conda environment).
     ```
     conda install -c conda-forge graphviz
     ```
     You can also use other method to install it based on your OS: [click here for details](https://graphviz.org/download/).
   * **SFML Library**: Install the SFML library, which is used for grid-world visualization. You can typically install it using your system's package manager.
     For MacOS, you can use Homebrew:
     ```
     brew install sfml
     ```
3. Configuring the Makefile\
   For the provided Makefile to work properly, you should adjust some paths to match the locations of libraries on your system. Here are the lines you should check and potentially modify:
   ```
   CXXFLAGS = ... -I$(HOME)/miniconda3/envs/spotenv/include -I/usr/local/include
   LDFLAGS = ... -L$(HOME)/miniconda3/envs/spotenv/lib ... -L/usr/local/lib
   ```
   * **Spot Library**:\
      If you installed Spot using a method other than conda, or if you installed it in a different conda environment than spotenv, you'll need to adjust the include (-I) and library (-L) paths in CXXFLAGS and LDFLAGS respectively:
     * Modify the path in **-I$(HOME)/miniconda3/envs/spotenv/include** to point to where Spot's header files are located on your system.
     * Similarly, adjust **-L$(HOME)/miniconda3/envs/spotenv/lib** to point to where Spot's library files are.
   * **SFML Library**:\
     The Makefile assumes SFML is installed in /usr/local/lib. If you installed it elsewhere, modify **-L/usr/local/lib** in LDFLAGS to point to the correct location.

Once you've made these adjustments, you should be able to run ```make``` to compile the project.

4. Adjusting the **run.sh** File\
Before executing your program with run.sh, ensure the script knows where to find the dynamic libraries used by the program.\
Here's what you need to adjust:

In the run.sh file, there's a line:
```
export DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH:$HOME/miniconda3/envs/spotenv/lib
```
This sets the dynamic linker to search for dynamic libraries in the specified path. Adjust the path $HOME/miniconda3/envs/spotenv/lib if you installed Spot in a different location or using a different conda environment than **spotenv**.\
After making this adjustment, grant execute permissions to the run.sh script using:
```
chmod +x run.sh
```

## Usage
Clean, compile and run the main.cpp file, by following these steps:
```
make clean
make
./run.sh
```
The program initializes a grid-world domain, defines locations for atomic propositions like goals, checkpoints, and hazardous areas, and then defines a list of LTLf formulas that dictate temporal tasks.

Upon running, the program attempts to find paths that satisfy each LTLf formula in the list, visualizing the path in the grid-world using the SFML library.

## Code Structure

* **GridState.h**, **GridWorldDomain.h**, **GridWorldPlotter.h**: Define the grid-world domain, states, and visualization methods respectively.
* **LTLFormula.h**: Provides functionality to handle and parse LTLf formulas.
* **TEGTask.h**: Handles tasks defined by LTLf formulas and attempts to solve them.
