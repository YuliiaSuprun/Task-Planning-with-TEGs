#include "GridWorldPlotter.h"

GridWorldPlotter::GridWorldPlotter(GridWorldDomain grid_domain, int windowWidth, int windowHeight) : grid_domain_{grid_domain}, grid_size_{grid_domain.R()}, WINDOW_WIDTH{windowWidth}, WINDOW_HEIGHT{windowHeight} {
    if (grid_size_ <= 0) {
        cerr << "ERROR: invalid grid size value!" << endl;
        return;
    }
    // Calculate cell size based on window size and grid dimensions
    cell_width_ = WINDOW_WIDTH / static_cast<float>(grid_size_);
    cell_height_ = WINDOW_HEIGHT / static_cast<float>(grid_size_);
}

void GridWorldPlotter::visualize_path(const TEGProblem& problem) {
    // Create an SFML window
    sf::RenderWindow window(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Grid Path Visualization");

    // Create an off-screen RenderTexture
    sf::RenderTexture renderTexture;
    if (!renderTexture.create(WINDOW_WIDTH, WINDOW_HEIGHT)) {
        cerr << "ERROR: cannot render a texture" << endl;
        return;
    }

    // Set up font and text for "start" label
    sf::Font font;
    // Load a font file for this. 
    if (!font.loadFromFile("font/Lato-Bold.ttf")) {
        cerr << "ERROR: no font file was found" << endl;
        return; // handle error
    }
    sf::Text startText("s", font, cell_width_ * 0.8); // Adjust the font size according to your needs.
    startText.setFillColor(sf::Color::Black);


    // Clear the renderTexture
    renderTexture.clear(sf::Color::White);

    // Draw the grid and obstacles to renderTexture
    for (size_t i = 0; i < grid_size_; ++i) {
        for (size_t j = 0; j < grid_size_; ++j) {
            sf::RectangleShape cell(sf::Vector2f(cell_width_, cell_height_));
            cell.setPosition(j * cell_width_, i * cell_height_);
            cell.setOutlineColor(sf::Color::Black);
            cell.setOutlineThickness(1.0f);

            GridState curr_state(i, j);

            if (grid_domain_.is_obstacle(curr_state)) {
                cell.setFillColor(sf::Color::Black);
            } else {
                cell.setFillColor(sf::Color::White);
            }
            renderTexture.draw(cell);
        }
    }

    auto grid_path = problem.get_grid_path();

    // Draw the path to renderTexture
    for (size_t i = 0; i < grid_path.size(); ++i) {
        const auto& state = grid_path[i];
        sf::RectangleShape pathCell(sf::Vector2f(cell_width_, cell_height_));
        pathCell.setPosition(state.y() * cell_width_, state.x() * cell_height_);
        if (state.isCached()) {
            pathCell.setFillColor(sf::Color::Magenta);
        } else {
            pathCell.setFillColor(sf::Color::Blue);
        }

        if (i == 0) { // This is the start cell
            pathCell.setFillColor(sf::Color::Green);
        }
        renderTexture.draw(pathCell);

        if (i == 0) {
            // Position and draw the "start" label
            startText.setPosition((state.y() + 0.3) * cell_width_,
                                  (state.x() - 0.2) * cell_height_);
            renderTexture.draw(startText);
        }
    }

    // Draw the label-color mapping
    auto labelToColorMapping = problem.get_ap_mapping();
    for (const auto& pair : labelToColorMapping) {
        const std::string& label = pair.first;
        for (const auto& gridState : pair.second) {
            sf::RectangleShape labelCell(sf::Vector2f(cell_width_, cell_height_));
            labelCell.setPosition(gridState.y() * cell_width_, gridState.x() * cell_height_);
            
            if (label == "g") {
                labelCell.setFillColor(sf::Color::Cyan);
            } else if (!label.empty() && label.at(0) == 'c') {
                labelCell.setFillColor(sf::Color::Yellow);
            } else if (label == "h") {
                labelCell.setFillColor(sf::Color::Red);
            }// Add more else-if conditions if want to have more labels

            renderTexture.draw(labelCell);

            sf::Text labelText(label, font, cell_width_ * 0.8);
            labelText.setFillColor(sf::Color::Black);
            labelText.setPosition((gridState.y() + 0.3) * cell_width_,
                                  (gridState.x() - 0.2) * cell_height_);
            renderTexture.draw(labelText);
        }
    }


    // Finalize the drawing to the renderTexture
    renderTexture.display();

    // Save the contents of renderTexture to an image
    sf::Image screenshot = renderTexture.getTexture().copyToImage();
    screenshot.saveToFile(problem.get_filename() + "path.png");

    // Draw the contents of the renderTexture to the main window for visualization
    sf::Sprite sprite(renderTexture.getTexture());
    window.draw(sprite);
    window.display();

    // Finally, run the event loop to handle window interactions.
    while (window.isOpen()) {
        sf::Event event;
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }
    }
}
