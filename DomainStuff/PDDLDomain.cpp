#include "PDDLDomain.h"
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>

PDDLDomain::PDDLDomain(const string& domainFile) {
    parseDomain(domainFile);
    cout << "Domain was parsed!!!" << endl;
    pddlDomain_->toPDDL(std::cout) << std::endl;
}

void PDDLDomain::parseDomain(const string& domainFile) {
    pddlboat::ast::Domain domain_ast;
    if (!pddlboat::parseFile(domainFile, domain_ast)) {
        throw runtime_error("Failed to parse PDDL domain file");
    }

    try {
        pddlDomain_ = pddlboat::toDomain(domain_ast);
    } catch (const exception& e) {
        throw runtime_error("Exception translating domain: " + string(e.what()));
    }
}

// Implementations of pure virtual functions from Domain

// Placeholder for get_actions function
const vector<shared_ptr<PrimitiveAction>>& PDDLDomain::get_actions(const DomainState& state) const {
    static vector<shared_ptr<PrimitiveAction>> actions;
    // Placeholder logic, return an empty vector
    return actions;
}

// Placeholder for get_all_states function
const vector<shared_ptr<DomainState>>& PDDLDomain::get_all_states() {
    static vector<shared_ptr<DomainState>> states;
    // Placeholder logic, return an empty vector
    return states;
}

// Placeholder for get_successor_states function
const vector<shared_ptr<DomainState>> PDDLDomain::get_successor_states(const DomainState& curr_state) {
    vector<shared_ptr<DomainState>> successor_states;
    // Placeholder logic, return an empty vector
    return successor_states;
}

// Placeholder for get_successor_state_action_pairs function
const vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> PDDLDomain::get_successor_state_action_pairs(const DomainState& state) {
    vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> state_action_pairs;
    // Placeholder logic, return an empty vector
    return state_action_pairs;
}

// Placeholder for is_valid_state function
bool PDDLDomain::is_valid_state(const DomainState& state) const {
    // Placeholder logic, return false
    return false;
}

// Placeholder for mark_as_explored function
void PDDLDomain::mark_as_explored(const DomainState& state) {
    // Placeholder logic, do nothing
}

// Placeholder for mark_all_states_as_unexplored function
void PDDLDomain::mark_all_states_as_unexplored() {
    // Placeholder logic, do nothing
}

std::shared_ptr<pddlboat::Domain> PDDLDomain::getPddlboatDomainPtr() const {
    return pddlDomain_;
}
