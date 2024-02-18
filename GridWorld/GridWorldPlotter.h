#ifndef GRIDWORLDPLOTTER_H
#define GRIDWORLDPLOTTER_H

#include "TEGProblem.h"
#include "GridWorldDomain.h"
#include <SFML/Graphics.hpp>

class GridWorldPlotter {
public:
    GridWorldPlotter(GridWorldDomain grid_domain, int windowWidth=800, int windowHeight=800);

    void visualize_path(const TEGProblem& task);

private:

    GridWorldDomain grid_domain_;
    size_t grid_size_;
    const int WINDOW_WIDTH;
    const int WINDOW_HEIGHT;

    float cell_width_;
    float cell_height_;
};

#endif // GRIDWORLDPLOTTER_H
