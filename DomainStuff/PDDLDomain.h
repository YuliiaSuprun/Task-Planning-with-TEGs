#ifndef PDDL_DOMAIN_H
#define PDDL_DOMAIN_H

#include <pddlboat/solver/domain.hpp>
#include "Domain.h"
#include "PDDLAction.h"
#include "PDDLState.h"
#include <string>
#include <memory>

using namespace std;

class PDDLDomain : public Domain {
public:
    explicit PDDLDomain(const string& domainFile);
    virtual ~PDDLDomain() = default;

    // Implement pure virtual functions from Domain
    // virtual const vector<shared_ptr<PrimitiveAction>>& get_actions(const DomainState& state) const override;
    virtual const vector<shared_ptr<DomainState>>& get_all_states() override;
    virtual const vector<shared_ptr<DomainState>> get_successor_states(const DomainState& curr_state) override;
    virtual const vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> get_successor_state_action_pairs(const DomainState& state) override;
    virtual bool is_valid_state(const DomainState& state) const override;
    virtual void mark_as_explored(const DomainState& state) override;
    virtual void mark_all_states_as_unexplored() override;

    // Other functions specific to PDDLDomain
    shared_ptr<pddlboat::Domain> getPddlboatDomainPtr() const;

    void setProblem(pddlboat::ProblemPtr problemPtr);
    static pddlboat::DomainPtr parseDomain(const string& domainFile);

private:
    pddlboat::DomainPtr pddlDomain_;
    pddlboat::ProblemPtr pddlProblem_;
    vector<shared_ptr<PDDLAction>> actions_;
    void initializeActions();
    // pddlboat::StatePtr createInitialState(const pddlboat::ProblemPtr& problem);

    // bool isOfType(const string& objName, const string& type);
};

#endif // PDDL_DOMAIN_H
