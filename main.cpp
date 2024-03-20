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

using namespace std;


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
        // pddlProblem = make_shared<PDDLProblem>(problemFilePath, pddlDomain);
    } catch (const std::exception& e) {
        cerr << "Error: " << e.what() << endl;
        exit(EXIT_FAILURE);
    }

    return 0;
}