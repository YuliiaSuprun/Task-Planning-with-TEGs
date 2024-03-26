#include "PDDLProblem.h"
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>
#include <iostream>

PDDLProblem::PDDLProblem(const string& problemFile, shared_ptr<PDDLDomain> domainPtr, bool cache, bool feedback, bool use_landmarks, int problem_id) 
: domain_(domainPtr), problem_id_(problem_id), cache_(cache),
    use_landmarks_(use_landmarks),
    bdd_dict_(make_shared<spot::bdd_dict>()),
    domain_manager_(make_shared<DomainManager>(bdd_dict_, domain_)),
    dfa_manager_(make_shared<DFAManager>(bdd_dict_, domain_manager_->get_all_equivalence_regions(), feedback)),
    product_manager_(make_shared<ProductManager>(domain_manager_, dfa_manager_)) {
    parseProblem(problemFile, domainPtr);
    cout << "Problem was parsed!!!" << endl;
    pddlProblem_->toPDDL(std::cout) << std::endl;

    // Set a start position.
    start_domain_state_ = make_shared<PDDLState>(pddlProblem_->start);

    std::cout << "Creating dfa for problem_id=" << problem_id_ << endl;
    // Get a name for output files.
    stringstream ss;
    ss << "pddlDfa" << problem_id_;
    filename_ = ss.str();

    auto start1 = chrono::high_resolution_clock::now();

    // Convert to formula_str to LTLFormula object.
    formula_ = LTLFormula(pddlProblem_->goal->toLTL());

    // Compute the DFA corresponding to the given LTL formula.
    dfa_manager_->construct_dfa(formula_);

    auto end1 = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_dfa = end1 - start1;
    std::cout << "time of calculating the automaton: " << elapsed_dfa.count() << " seconds" << std::endl;

    // dfa_manager_->print_dfa();
    dfa_manager_->save_dfa(filename_);
}

void PDDLProblem::parseProblem(const std::string& problemFile, std::shared_ptr<PDDLDomain> domainPtr) {
    auto pddlboatDomainPtr = domainPtr->getPddlboatDomainPtr();

    pddlboat::ast::Problem problem_ast;
    if (!pddlboat::parseFile(problemFile, problem_ast)) {
        throw std::runtime_error("Failed to parse PDDL problem file: " + problemFile);
    }

    try {
        pddlProblem_ = pddlboat::toProblem(problem_ast, pddlboatDomainPtr);
    } catch (const std::exception& e) {
        throw std::runtime_error("Exception translating problem: " + std::string(e.what()));
    }
}

const pddlboat::ProblemPtr& PDDLProblem::getPddlboatProblemPtr() const {
    return pddlProblem_;
}