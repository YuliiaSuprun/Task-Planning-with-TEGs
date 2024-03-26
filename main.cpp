#include <iostream>
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

// ./run.sh /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/pddlboat/resources/blocks/blockworld.pddl /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/pddlboat/resources/blocks/p00.pddl

// Temporal
// ./run.sh /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/domain.pddl /Users/yuliiasuprun/Desktop/Classes/AlgoRobotics/Research/Code/Plan4Past-data/deterministic/LTLf/TB15/blocksworld/c03.pddl


int main(int argc, char** argv) {
    if (argc < 3) {
        cerr << "Usage: " << argv[0] << " <PDDL Domain File> <PDDL Problem File>" << endl;
        exit(EXIT_FAILURE);
    }

    string domainFilePath = argv[1];
    string problemFilePath = argv[2];

    // Instantiate PDDLDomain and PDDLProblem.
    shared_ptr<PDDLDomain> pddlDomain;
    shared_ptr<PDDLProblem> pddlProblem;
    try {
        pddlDomain = make_shared<PDDLDomain>(domainFilePath);
        pddlProblem = make_shared<PDDLProblem>(problemFilePath, pddlDomain);
    } catch (const std::exception& e) {
        cerr << "Error: " << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    auto pddlboatProblem = pddlProblem->getPddlboatProblemPtr();
    // Do not forget to set a problem!!
    pddlDomain->setProblem(pddlboatProblem);

    // Step 1: Convert problem goal into LTLf formula as a string.
    pddlboat::ExpressionPtr goalExpression = pddlboatProblem->goal;
    cout << "The goal is defined as follows: " << endl;
    goalExpression->toPDDL(std::cout) << std::endl;

    auto ltlStr = goalExpression->toLTL();
    cout << "The goal in the LTL format: " << ltlStr << endl;

    auto startState = pddlboatProblem->start; 
    std::vector<pddlboat::State::Key> allTruePreds = startState->getTruePredicates();
    for (const auto& key : allTruePreds) {
        cout << "Predicate is " << key.first << " with assignment: " << endl;
        for (const auto& value : key.second) {
            cout << value << "; ";
        }
        cout << "=====" << endl;
    }

    // Step 2: Generate a mapping from all grounded predicates to atomic propositions.

    // Step 3: Pass a string LTLf formula and ap_mapping to the TEGProblem and solve it!

    // Solve the problem
    pddlboat::Z3Planner::Options options;
    // options.dump_clauses = true;
    // options.horizon.max = 4;
    // auto task_planner = make_shared<pddlboat::Z3Planner>(pddlboatProblem, options);
    auto task_planner = make_shared<pddlboat::AStarPlanner>(pddlboatProblem);

    cout << "Plan:" << endl;
    auto plan = make_shared<pddlboat::Plan>(pddlboatProblem);
    if (!task_planner->solve(*plan))
    {
        cerr << "Failed to solve!" << endl;
        return 1;
    }
    cout << *plan << endl;
    return 0;
}