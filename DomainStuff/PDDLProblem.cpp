#include "PDDLProblem.h"
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>
#include <iostream>

PDDLProblem::PDDLProblem(const std::string& problemFile, std::shared_ptr<PDDLDomain> domainPtr) {
    parseProblem(problemFile, domainPtr);
    cout << "Problem was parsed!!!" << endl;
    pddlProblem_->toPDDL(std::cout) << std::endl;
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