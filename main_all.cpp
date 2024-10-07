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
// ./run.sh /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld 1 -f -h --save_dfa --planner fd --search lama

// Function to print stats to any output stream (either console or file)
void printStats(std::ostream& os, int numRuns, double averageDFATime, double firstRunDFATime, double averageDFATimeNoFirst, double averageSearchTime, double averageTotalTime, double averageExpandedNodes, double averagePlanLength, double averageNumOfBacktracks) {
    os << "For " << numRuns << " runs: " << std::endl;
    os << "Average DFA construction time: " << averageDFATime << " seconds" << std::endl;
    os << "First DFA construction time: " << firstRunDFATime << " seconds" << std::endl;
    os << "Average DFA construction time (without first): " << averageDFATimeNoFirst << " seconds" << std::endl;
    os << "Average search time: " << averageSearchTime << " seconds" << std::endl;
    os << "Average total time: " << averageTotalTime << " seconds" << std::endl;
    os << "Average number of expanded nodes: " << averageExpandedNodes << std::endl;
    os << "Average plan length: " << averagePlanLength << std::endl;
    os << "Average number of backtracks: " << averageNumOfBacktracks << std::endl;
}

int main(int argc, char** argv) {
    if (argc < 3) { 
        cerr << "Usage: " << argv[0] << " <Directory Path> <Number of Runs> [-c] [-f] [-l] [-h] [--save_dfa] [--planner <planner_type>] [--search <search_type> (used only for the FD planner)] [--timeout <in ms>]" << endl;
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
    bool save_dfa = false;
    string planner_type;
    string search_type;
    string name_connector = "_"; // Default value for name connector
    // Domain specific: "_" for Rovers, "-" for Openstacks
    if (directoryPath.find("openstacks") != std::string::npos) {
        name_connector = "-";
    } else if (directoryPath.find("rovers") != std::string::npos) {
        name_connector = "_";
    }
    size_t subproblem_timeout = 60000; // 1 minute by default

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
        } else if (arg == "--save_dfa") {
            save_dfa = true;
        } else if (arg == "--planner" && i + 1 < argc) {
            planner_type = argv[++i]; // Move to the next argument
        } else if (arg == "--search" && i + 1 < argc) {
            search_type = argv[++i]; // Move to the next argument
        } else if (arg == "--timeout" && i + 1 < argc) {
            subproblem_timeout = stoi(argv[++i]); // Move to the next argument
        } else {
            cerr << "Unknown argument: " << arg << endl;
            exit(EXIT_FAILURE);
        }
    }

    // Validate planner type
    if (planner_type.empty()) {
        cerr << "No planner type provided. Use --planner <planner_type>. Options: manual, z3, fd, ff, symk, astar" << endl;
        exit(EXIT_FAILURE);
    } else {
        cout << "Using the planner: " << planner_type << endl;
    }

    if (planner_type == "fd" && search_type.empty()) {
        cerr << "No search type provided for the FD planner. Use --search <search_type>. Options: lazygreedy, fdat, lama" << endl;
        exit(EXIT_FAILURE);
    } else if (planner_type == "fd") {
        cout << "Using the search: " << search_type << endl;
    }

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

        if (problemFilePath.find("icloud") != string::npos) {
            // Some Mac files....
            continue;
        }
        cout << "Solving " << problemFilePath << "..." << endl;

        // Extract a problem name.
        string problem_name = PDDLProblem::extract_problem_name(problemFilePath);
        string domain_name = PDDLProblem::extract_domain_name(problemFilePath);

        string dir_name = "tide_solutions";
        // Create the solutions directory if it doesn't exist
        filesystem::create_directory(dir_name);

        // Create the planner subdirectory if it doesn't exist
        dir_name = dir_name + "/" + planner_type + search_type;
        filesystem::create_directory(dir_name);

        // Create the domain subdirectory if it doesn't exist
        dir_name = dir_name + "/" + domain_name;
        filesystem::create_directory(dir_name);

        // Create the problem subdirectory if it doesn't exist
        dir_name = dir_name + "/" + problem_name;
        filesystem::create_directory(dir_name);

        // Variables to store total elapsed time for averaging
        double totalDFATime = 0;
        double firstRunDFATime = 0;
        double DFATimeWithoutFirst = 0;
        double totalSearchTime = 0;

        size_t totalExpandedNodes = 0;
        size_t totalPlanLength = 0;
        size_t totalNumOfBacktracks = 0;

        for (int run = 0; run < numRuns; ++run) {
            cout << "Run #" << run + 1 << " for " << problemFilePath << endl;
            // Instantiate PDDLDomain and PDDLProblem.
            shared_ptr<PDDLDomain> pddlDomain;
            shared_ptr<PDDLProblem> pddlProblem;
            try {
                pddlDomain = make_shared<PDDLDomain>(domainFilePath);
                pddlProblem = make_shared<PDDLProblem>(problemFilePath, pddlDomain, planner_type, search_type, name_connector, cache, feedback, use_landmarks, hamming_dist, save_dfa, subproblem_timeout);
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
            totalNumOfBacktracks += pddlProblem->get_num_of_backtracks();
            
            cout << "Run " << (run + 1) << " DFA Time: " << dfa_time << " Search Time: " << elapsed.count() << " seconds" << endl;

            if (!solution_path.empty()) {
                string planFilePath = dir_name + "/" + problem_name + "_plan_" + to_string(run + 1);
                pddlProblem->write_solution_to_file(planFilePath);
                // pddlProblem->print_product_path();
                // pddlProblem->print_domain_path();
                pddlProblem->print_dfa_path();
                totalPlanLength += (pddlProblem->get_plan_length());
            } else {
                cout << "No solution found for " << problemFilePath << endl;
            }
        }
        double averageDFATime = totalDFATime / numRuns;
        double averageSearchTime = totalSearchTime / numRuns;
        double averageTotalTime = averageDFATime + averageSearchTime;
        double averageDFATimeNoFirst = (numRuns > 1) ? (DFATimeWithoutFirst / (numRuns-1)) : 0;
        double averageExpandedNodes = static_cast<double>(totalExpandedNodes) / numRuns;
        double averagePlanLength = static_cast<double>(totalPlanLength) / numRuns;
        double averageNumOfBacktracks = static_cast<double>(totalNumOfBacktracks) / numRuns;
        printStats(std::cout, numRuns, averageDFATime, firstRunDFATime, averageDFATimeNoFirst, averageSearchTime, averageTotalTime, averageExpandedNodes, averagePlanLength, averageNumOfBacktracks);

        // Create the stats file
        string statsFilePath = dir_name + "/stats.txt";
        ofstream statsFile(statsFilePath);

        // Check if the file is open before writing
        if (!statsFile.is_open()) {
            cerr << "Failed to open stats file: " << statsFilePath << endl;
            return EXIT_FAILURE;
        }

        // Print statistics to the file
        printStats(statsFile, numRuns, averageDFATime, firstRunDFATime, averageDFATimeNoFirst, averageSearchTime, averageTotalTime, averageExpandedNodes, averagePlanLength, averageNumOfBacktracks);

        // Close the file
        statsFile.close();
    }

    return 0;
}