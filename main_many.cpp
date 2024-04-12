#include <iostream>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <utility>
#include <chrono>
#include <filesystem> 

#include "GridState.h"
#include "GridWorldDomain.h"
#include "GridWorldPlotter.h"

#include "TEGProblem.h"
#include "PDDLDomain.h"
#include "PDDLProblem.h"
#include <pddlboat/utility.hpp>
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>
#include <pddlboat/parser/ast.hpp>
#include <pddlboat/solver/expressions.hpp>
#include <pddlboat/solver/domain.hpp>
#include <pddlboat/solver/planner.hpp>
#include <pddlboat/solver/fdplanner.hpp>
#include <pddlboat/solver/z3planner.hpp>
#include <pddlboat/solver/astarplanner.hpp>

using namespace std;
namespace fs = std::filesystem;

// Example of the command
// ./run.sh /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/domain.pddl /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/e03.pddl -f -c -h


int main(int argc, char** argv) {
    if (argc < 2) {
        cerr << "Usage: " << argv[0] << " <Directory Path> [-c] [-f] [-l] [-h]" << endl;
        exit(EXIT_FAILURE);
    }

    string directoryPath = argv[1];
    bool cache = false;
    bool feedback = false;
    bool use_landmarks = false;
    bool hamming_dist = false;

    for (int i = 2; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-c") {
            cache = true;
        } else if (arg == "-f") {
            feedback = true;
        } else if (arg == "-l") {
            use_landmarks = true;
        } else if (arg == "-h") {
            hamming_dist = true;
        } else {
            cerr << "Unknown argument: " << arg << endl;
            exit(EXIT_FAILURE);
        }
    }

    // Instantiate PDDLDomain.
    string domainFilePath = directoryPath + "/domain.pddl";
    shared_ptr<PDDLDomain> pddlDomain;
     try {
        pddlDomain = make_shared<PDDLDomain>(domainFilePath);
    } catch (const std::exception& e) {
        cerr << "Error: " << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    int problem_id = 0;

    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        if (entry.is_regular_file()) {
            string problemFilePath = entry.path().string();
            cout << "Solving " << problemFilePath << "..." << endl;
            // Skip the domain file
            if (problemFilePath.find("domain.pddl") == string::npos) { 
                // Instantiate PDDLProblem.
                shared_ptr<PDDLProblem> pddlProblem;
                try {
                    pddlProblem = make_shared<PDDLProblem>(problemFilePath, pddlDomain, cache, feedback, use_landmarks, hamming_dist, problem_id);
                } catch (const std::exception& e) {
                    cerr << "Error: " << e.what() << endl;
                    exit(EXIT_FAILURE);
                }

                auto pddlboatProblem = pddlProblem->getPddlboatProblemPtr();
                // Do not forget to set a problem!!
                pddlDomain->setProblem(pddlboatProblem);

                auto start = chrono::high_resolution_clock::now();

                // Solve the problem and get the solution path
                vector<ProductState> solution_path = pddlProblem->solve();

                auto end = chrono::high_resolution_clock::now();

                if (!solution_path.empty()) {
                    cout << "Solution for problem " << problem_id << " was found" << endl;
                    // pddlProblem->print_product_path();
                    // pddlProblem->print_domain_path();
                    // pddlProblem->print_dfa_path();
                } else {
                    cout << "No solution for problem " << problem_id << " was found" << endl;
                }

                chrono::duration<double> elapsed = end - start;
                std::cout << "Time of searching for a solution: " << elapsed.count() << " seconds" << std::endl;
                problem_id++;
            }
        }
    }

    return 0;
}