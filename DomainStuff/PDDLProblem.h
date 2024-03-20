#ifndef PDDL_PROBLEM_H
#define PDDL_PROBLEM_H

#include "PDDLDomain.h" 

class PDDLProblem {
public:
    explicit PDDLProblem(const std::string& problemFile, std::shared_ptr<PDDLDomain> domainPtr);
    virtual ~PDDLProblem() = default;

private:
    std::shared_ptr<pddlboat::Problem> pddlProblem_;
    void parseProblem(const std::string& problemFile, std::shared_ptr<PDDLDomain> domainPtr);
};

#endif // PDDL_PROBLEM_H
