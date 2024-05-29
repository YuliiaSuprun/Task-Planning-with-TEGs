#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <utility>
#include <chrono>
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

// Example of the command
// ./run.sh /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/domain.pddl /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/a03.pddl 3 -f -c -h


int main(int argc, char** argv) {
    if (argc < 3) { 
        cerr << "Usage: " << argv[0] << " <Directory Path> <Number of Runs> [-c] [-f] [-l] [-h] [-p]" << endl;
        exit(EXIT_FAILURE);
    }

    // string domainFilePath = argv[1];
    // string problemFilePath = argv[2];
    string directoryPath = argv[1];
    int numRuns = stoi(argv[2]);

    bool cache = false;
    bool feedback = false;
    bool use_landmarks = false;
    bool hamming_dist = false;
    bool use_planner = false;

    for (int i = 3; i < argc; ++i) {
        string arg = argv[i];
        if (arg == "-c") {
            cache = true;
        } else if (arg == "-f") {
            feedback = true;
        } else if (arg == "-l") {
            use_landmarks = true;
        } else if (arg == "-h") {
            hamming_dist = true;
        }  else if (arg == "-p") {
            use_planner = true;
        } else {
            cerr << "Unknown argument: " << arg << endl;
            exit(EXIT_FAILURE);
        }
    }

    // Variables to store total elapsed time and for averaging
    double totalDFATime = 0;
    double firstRunDFATime = 0;
    double DFATimeWithoutFirst = 0;
    double totalSearchTime = 0;

    size_t totalExpandedNodes = 0;
    size_t totalPlanLength = 0;
    string solution_file_path;

    string domainFilePath = directoryPath + "/domain.pddl";

    for (const auto& entry : fs::directory_iterator(directoryPath)) {
        if (!entry.is_regular_file()) {
            // Skip it.
            continue;
        }
        string problemFilePath = entry.path().string();
        if (problemFilePath.find("domain.pddl") != string::npos) {
            // This is the domian file. Skip it.
            continue;
        }
        cout << "Solving " << problemFilePath << "..." << endl;

        for (int run = 0; run < numRuns; ++run) {
            // Instantiate PDDLDomain and PDDLProblem.
            shared_ptr<PDDLDomain> pddlDomain;
            shared_ptr<PDDLProblem> pddlProblem;
            try {
                pddlDomain = make_shared<PDDLDomain>(domainFilePath);
                pddlProblem = make_shared<PDDLProblem>(problemFilePath, pddlDomain, cache, feedback, use_landmarks, hamming_dist, use_planner, run==0);
            } catch (const std::exception& e) {
                cerr << "Error: " << e.what() << endl;
                exit(EXIT_FAILURE);
            }

            auto pddlboatProblem = pddlProblem->getPddlboatProblemPtr();
            // Do not forget to set a problem!!
            pddlDomain->setProblem(pddlboatProblem);

            double dfa_time = pddlProblem->get_dfa_construction_time();
            if (run == 0) {
                firstRunDFATime = dfa_time;
            } else {
                DFATimeWithoutFirst += dfa_time;
            }
            totalDFATime += dfa_time;

            auto start = chrono::high_resolution_clock::now();

            // Solve the problem and get the solution path
            vector<ProductState> solution_path = pddlProblem->solve();

            auto end = chrono::high_resolution_clock::now();
            chrono::duration<double> elapsed = end - start;

            totalSearchTime += elapsed.count();
            totalExpandedNodes += pddlProblem->get_num_expanded_nodes();
            
            cout << "Run " << (run + 1) << " DFA Time: " << dfa_time << " Search Time: " << elapsed.count() << " seconds" << endl;

            if (!solution_path.empty()) {
                // cout << "Writing solution to a file..." << endl;
                if (run == 0) {
                    solution_file_path = pddlProblem->write_solution_to_file();
                }
                // pddlProblem->print_product_path();
                // pddlProblem->print_domain_path();
                pddlProblem->print_dfa_path();
                totalPlanLength += (pddlProblem->get_domain_path().size()-1);
            } else {
                cout << "No solution found for problem" << endl;
            }
        }
        double averageDFATime = totalDFATime / numRuns;
        double averageSearchTime = totalSearchTime / numRuns;
        double averageTotalTime = averageDFATime + averageSearchTime;
        double averageDFATimeNoFirst = (numRuns > 1) ? (DFATimeWithoutFirst / (numRuns-1)) : 0;
        double averageExpandedNodes = static_cast<double>(totalExpandedNodes) / numRuns;
        double averagePlanLength = static_cast<double>(totalPlanLength) / numRuns;
        cout << "For " << numRuns << " runs: " << endl;
        cout << "Average DFA construction time: " << averageDFATime << " seconds" << endl;
        cout << "First DFA construction time: " << firstRunDFATime << " seconds" << endl;
        cout << "Average DFA construction time (without first): " << averageDFATimeNoFirst << " seconds" << endl;
        cout << "Average search time: " << averageSearchTime << " seconds" << endl;
        cout << "Average total time: " << averageTotalTime << " seconds" << endl;
        cout << "Average number of expanded nodes: " << averageExpandedNodes << endl;
        cout << "Average plan length: " << averagePlanLength << endl;

        if (!solution_file_path.empty()) {

            // Open the solution file for appending
            std::ofstream ofs(solution_file_path, std::ios_base::app);

            if (ofs.is_open()) {
                ofs << "For " << numRuns << " runs: " << std::endl;
                ofs << "Average DFA construction time: " << averageDFATime << " seconds" << std::endl;
                ofs << "First DFA construction time: " << firstRunDFATime << " seconds" << std::endl;
                ofs << "Average DFA construction time (without first): " << averageDFATimeNoFirst << " seconds" << std::endl;
                ofs << "Average search time: " << averageSearchTime << " seconds" << std::endl;
                ofs << "Average total time: " << averageTotalTime << " seconds" << std::endl;
                ofs << "Average number of expanded nodes: " << averageExpandedNodes << std::endl;
                ofs << "Average plan length: " << averagePlanLength << std::endl;

                // Close the file
                ofs.close();
            } else {
                std::cerr << "Failed to open file: " << solution_file_path << std::endl;
            }
        }
    }

    return 0;
}