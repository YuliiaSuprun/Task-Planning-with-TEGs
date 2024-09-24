#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <utility>
#include <chrono>
#include <stdexcept>
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
// ./run.sh /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/domain.pddl /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/a03.pddl 3 -f -h --save_dfa --planner fd --search lama

// Commands for debugging
// Translate 
// /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/pddlboat/submodules/downward/src/translate/translate.py --keep-unreachable-facts --keep-unimportant-variables --sas-file private/tmp/pddlboat_sas_7c6dd358-184b-4481-9ddc-5765ff878025 private/tmp/pddlboat_domain_7c6dd358-184b-4481-9ddc-5765ff878025.pddl private/tmp/pddlboat_problem_7c6dd358-184b-4481-9ddc-5765ff878025.pddl

// Solve:
// /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/pddlboat/build/release/submodules/downward/src/bin/downward --internal-plan-file private/tmp/pddlboat_out_7c6dd358-184b-4481-9ddc-5765ff878025 --evaluator "hff=ff()" --evaluator "hcea=cea()" --search "lazy_greedy([hff, hcea], preferred=[hff, hcea])" < private/tmp/pddlboat_sas_7c6dd358-184b-4481-9ddc-5765ff878025

// All together (working with axioms)
// /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/pddlboat/submodules/downward/fast-downward.py /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Task-Planning-with-TEGs/private/tmp/pddlboat_domain_7c6dd358-184b-4481-9ddc-5765ff878025.pddl /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Task-Planning-with-TEGs/private/tmp/pddlboat_problem_7c6dd358-184b-4481-9ddc-5765ff878025_full.pddl --evaluator "hff=ff()" --evaluator "hcea=cea()" --search "lazy_greedy([hff, hcea], preferred=[hff, hcea])"

// With constraints but it doesn't work because FD planner supports only PDDL2.2 functionality (constraints were introduced later) :
// /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/pddlboat/submodules/downward/fast-downward.py /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Task-Planning-with-TEGs/private/tmp/pddlboat_domain_7c6dd358-184b-4481-9ddc-5765ff878025_initial.pddl /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Task-Planning-with-TEGs/private/tmp/pddlboat_problem_7c6dd358-184b-4481-9ddc-5765ff878025_constraints.pddl --evaluator "hff=ff()" --evaluator "hcea=cea()" --search "lazy_greedy([hff, hcea], preferred=[hff, hcea])" 



int main(int argc, char** argv) {
    if (argc < 4) { 
        cerr << "Usage: " << argv[0] << " <PDDL Domain File> <PDDL Problem File> <Number of Runs> [-c] [-f] [-l] [-h] [--save_dfa] [--planner <planner_type>] [--search <search_type> (used only for the FD planner)] [--timeout <in ms>]" << endl;
        exit(EXIT_FAILURE);
    }

    string domainFilePath = argv[1];
    string problemFilePath = argv[2];
    int numRuns = stoi(argv[3]);

    bool cache = false;
    bool feedback = false;
    bool use_landmarks = false;
    bool hamming_dist = false;
    bool save_dfa = false;
    string planner_type;
    string search_type;
    string name_connector = "_"; // Default value for name connector
    // Domain specific: "_" for Rovers, "-" for Openstacks
    if (domainFilePath.find("openstacks") != std::string::npos) {
        name_connector = "-";
    } else if (domainFilePath.find("rovers") != std::string::npos) {
        name_connector = "_";
    }
    size_t subproblem_timeout = 60000; // 1 minute by default

    for (int i = 4; i < argc; ++i) {
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

    // Variables to store total elapsed time and for averaging
    double totalDFATime = 0;
    double firstRunDFATime = 0;
    double DFATimeWithoutFirst = 0;
    double totalSearchTime = 0;

    size_t totalExpandedNodes = 0;
    size_t totalPlanLength = 0;
    size_t totalNumOfBacktracks = 0;
    string solution_file_path;

    for (int run = 0; run < numRuns; ++run) {
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
    double averageNumOfBacktracks = static_cast<double>(totalNumOfBacktracks) / numRuns;
    cout << "For " << numRuns << " runs: " << endl;
    cout << "Average DFA construction time: " << averageDFATime << " seconds" << endl;
    cout << "First DFA construction time: " << firstRunDFATime << " seconds" << endl;
    cout << "Average DFA construction time (without first): " << averageDFATimeNoFirst << " seconds" << endl;
    cout << "Average search time: " << averageSearchTime << " seconds" << endl;
    cout << "Average total time: " << averageTotalTime << " seconds" << endl;
    cout << "Average number of expanded nodes: " << averageExpandedNodes << endl;
    cout << "Average plan length: " << averagePlanLength << endl;
    cout << "Average number of backtracks: " << averageNumOfBacktracks << endl;

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
            ofs << "Average number of backtracks: " << averageNumOfBacktracks << std::endl;

            // Close the file
            ofs.close();
        } else {
            std::cerr << "Failed to open file: " << solution_file_path << std::endl;
        }
    }

    return 0;
}