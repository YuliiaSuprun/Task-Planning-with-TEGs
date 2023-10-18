# GridWorld Planning with LTLf Formulas
This repository contains a C++ implementation of a planner that solves tasks in a grid-world environment using Linear Temporal Logic formulas (LTLf). The code represents grid-world domains, obstacles, states, and goals, and uses LTLf formulas to define temporal tasks.
## Installation

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
