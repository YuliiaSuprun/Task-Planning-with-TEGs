#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <set>
#include <map>
#include <string>
#include <utility>
#include <chrono>
#include <cstdlib>
#include <nlohmann/json.hpp>
#include <regex>

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
// ./run.sh /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/PPLTL/TB15/blocksworld /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/PPLTL/TB15/blocksworld/blocksworld_teg.json 1 --planner fd --search lama

// Example command to run plan4past:
// plan4past -d examples/pddl/domain.pddl -p examples/pddl/p-0.pddl -g "ontable_c & O(on_b_a)" -od output_domain.pddl -op output_problem.pddl

// Function to print stats to any output stream (either console or file)
void printStats(std::ostream& os, int numRuns, double averageTranslateTime, double averageSearchTime, double averageTotalTime, double averagePlanLength) {
    os << "For " << numRuns << " runs:" << std::endl;
    os << "Average translate time: " << averageTranslateTime << " seconds" << std::endl;
    os << "Average solve time: " << averageSearchTime << " seconds" << std::endl;
    os << "Average total time: " << averageTotalTime << " seconds" << std::endl;
    os << "Average plan length: " << averagePlanLength << std::endl;
}


int main(int argc, char** argv) {
    if (argc < 4) {
        cerr << "Usage: " << argv[0] << " <Directory Path> <Goal JSON File> <Number of Runs>" << endl;
        exit(EXIT_FAILURE);
    }

    string directoryPath = argv[1];
    string goalJsonFilePath = argv[2];
    int numRuns = stoi(argv[3]);
    string domainFilePath = directoryPath + "/domain.pddl";

    for (const auto& entry : filesystem::directory_iterator(directoryPath)) {
        if (!entry.is_regular_file()) {
            // Skip it.
            continue;
        }
        string problemFilePath = entry.path().string();
        if (problemFilePath.find("domain.pddl") != string::npos) {
            // This is the domian file. Skip it.
            continue;
        }
        if (problemFilePath.find(".json") != string::npos) {
            // This is the json file for PPLTL goals. Skip it.
            continue;
        }
        if (problemFilePath.find(".map") != string::npos) {
            // Weird map file. Skip it.
            continue;
        }

        if (problemFilePath.find("g02") != string::npos) {
            // Skip it.
            continue;
        }

        if (problemFilePath.find("g03") != string::npos) {
            // Skip it.
            continue;
        }

        cout << "Solving " << problemFilePath << "..." << endl;

        // Extract a problem name.
        string problem_name = PDDLProblem::extract_problem_name(problemFilePath);
        string domain_name = PDDLProblem::extract_domain_name(problemFilePath);
        // Load the goal expression from the JSON file
        string goal_expression;
        try {
            ifstream jsonFile(goalJsonFilePath);
            if (!jsonFile.is_open()) {
                cerr << "Error opening JSON file: " << goalJsonFilePath << endl;
                exit(EXIT_FAILURE);
            }

            // Parse the JSON file
            nlohmann::json goalData;
            jsonFile >> goalData;

            // Find the goal expression for the extracted problem name
            if (goalData.contains(problem_name)) {
                goal_expression = goalData[problem_name].get<string>();
                cout << "Goal expression for " << problem_name << ": " << goal_expression << endl;
            } else {
                cerr << "Problem name not found in the JSON file: " << problem_name << endl;
                exit(EXIT_FAILURE);
            }

        } catch (const std::exception &e) {
            std::cerr << "Error parsing JSON file: " << e.what() << std::endl;
            exit(EXIT_FAILURE);
        }


        // Variables to store total elapsed time for averaging
        double totalTranslateTime = 0;
        double totalSearchTime = 0;
        size_t totalPlanLength = 0;

        string dir_name = "plan4past_solutions";
        // Create the solutions directory if it doesn't exist
        filesystem::create_directory(dir_name);

        // Create the domain subdirectory if it doesn't exist
        dir_name = dir_name + "/" + domain_name;
        filesystem::create_directory(dir_name);

        // Create the problem subdirectory if it doesn't exist
        dir_name = dir_name + "/" + problem_name;
        filesystem::create_directory(dir_name);

        // Create transformed PDDL files
        string outputDomainPath = dir_name + "/output_domain.pddl";
        string outputProblemPath = dir_name + "/output_problem.pddl";

        string mapFilePath = directoryPath + "/" + problem_name + ".map";

        string mapFileArgString; // Empty by default
        if (filesystem::exists(mapFilePath)) {
            // If the map file exists, include it in the command
            mapFileArgString = " -m " + mapFilePath;
        } 
        // Construct `plan4past` command
        string plan4pastCmd = "plan4past -d " + domainFilePath + " -p " + problemFilePath + " -g \"" + goal_expression + "\"" + mapFileArgString + " -od " + outputDomainPath + " -op " + outputProblemPath + " > /dev/null 2>&1";
        // cout << "plan4pastCmd: " << plan4pastCmd << endl;

        // Construct the Fast Downward command
        string fastDownwardCmd = "/Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/pddlboat/submodules/downward/fast-downward.py --alias seq-sat-lama-2011 " + outputDomainPath + " " + outputProblemPath + " > /dev/null 2>&1";
        // cout << "fastDownwardCmd: " << fastDownwardCmd << endl;

        for (int run = 0; run < numRuns; ++run) {
            cout << "Run #" << run + 1 << " for " << problemFilePath << endl;
            // Step 1: Translate into the classical problem with P4P
            auto translate_start = chrono::high_resolution_clock::now();
            system(plan4pastCmd.c_str());  // Run the command
            auto translate_end = chrono::high_resolution_clock::now();
            chrono::duration<double> translate_elapsed = translate_end - translate_start;

            // Output the time taken to translate into a classical problem
            cout << "Plan4Past Translation time: " << translate_elapsed.count() << " seconds" << endl;

            // Step 2: Generate a solution running a classical planner (FD - LAMA)
            string planFilePath = dir_name + "/" + problem_name + "_plan_" + to_string(run + 1);

            // Run the Fast Downward command
            auto fd_start = chrono::high_resolution_clock::now();
            system(fastDownwardCmd.c_str());  // Execute the Fast Downward command

            // Measure time
            auto fd_end = chrono::high_resolution_clock::now();
            chrono::duration<double> fd_elapsed = fd_end - fd_start;

            // Output the time taken
            cout << "Fast Downward execution time: " << fd_elapsed.count() << " seconds" << endl;

            // Check if the Fast Downward plan was generated
            ifstream sasPlanFile("sas_plan.1");
            if (!sasPlanFile.is_open()) {
                cerr << "Failed to generate a plan for run #" << run + 1 << endl;
            } else {
                // Open the destination file for the plan
                ofstream outFile(planFilePath);
                if (!outFile.is_open()) {
                    cerr << "Failed to open output plan file: " << planFilePath << endl;
                } else {
                    string line;
                    // Process each line from the sas_plan.1 file
                    while (getline(sasPlanFile, line)) {
                        // Copy the contents from "sas_plan.1" to planFilePath
                        outFile << line << endl;

                        // Use a regex to match the line with the cost information
                        smatch match;
                        regex costRegex("; cost = ([0-9]+)");
                        if (regex_search(line, match, costRegex)) {
                            // Extract and convert the plan length
                            size_t planLength = stoi(match[1].str());
                            totalPlanLength += planLength; 
                            cout << "Plan length: " << planLength << endl;
                        }
                    }
                    outFile.close();
                }
                sasPlanFile.close();

                // Clean up the helper files generated by the FD planner.
                if (std::remove("sas_plan.1") != 0)
                {
                    cerr << "Failed to remove sas_plan.1 file." << endl;
                }
            }
            totalTranslateTime += translate_elapsed.count();
            totalSearchTime += fd_elapsed.count();
        }
        double averageTranslateTime = totalTranslateTime / numRuns;
        double averageSearchTime = totalSearchTime / numRuns;
        double averageTotalTime = (totalTranslateTime + totalSearchTime) / numRuns;
        double averagePlanLength = static_cast<double>(totalPlanLength) / numRuns;

        // Print statistics to stdout (console)
        printStats(std::cout, numRuns, averageTranslateTime, averageSearchTime, averageTotalTime, averagePlanLength);

        // Create the stats file
        string statsFilePath = dir_name + "/stats.txt";
        ofstream statsFile(statsFilePath);

        // Check if the file is open before writing
        if (!statsFile.is_open()) {
            cerr << "Failed to open stats file: " << statsFilePath << endl;
            return EXIT_FAILURE;
        }

        // Print statistics to the file
        printStats(statsFile, numRuns, averageTranslateTime, averageSearchTime, averageTotalTime, averagePlanLength);

        // Close the file
        statsFile.close();
    }

    return 0;
}