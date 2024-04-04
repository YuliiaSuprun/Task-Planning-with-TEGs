#include "PDDLDomain.h"
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>

PDDLDomain::PDDLDomain(const string& domainFile) {
    parseDomain(domainFile);
    cout << "Domain was parsed!!!" << endl;
    pddlDomain_->toPDDL(std::cout) << std::endl;

    initializeActions();
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

void PDDLDomain::initializeActions() {
    if (pddlDomain_) {
        for (const auto& pair : pddlDomain_->actions) {
            const auto& pddlAction = pair.second;
            auto primitiveAction = make_shared<PDDLAction>(pddlAction);
            actions_.push_back(primitiveAction); 
        }
    } else {
        throw runtime_error("Domain initialization failed");
    }
}

// Implementations of pure virtual functions from Domain

// const vector<shared_ptr<PrimitiveAction>>& PDDLDomain::get_actions(const DomainState& state) const {
//     // Actions are initialized during construction
//     return actions_;
// }

const vector<shared_ptr<DomainState>>& PDDLDomain::get_all_states() {
    static vector<shared_ptr<DomainState>> allPDDLStates;
    // Return an empty vector for now.
    return allPDDLStates;
}

const vector<shared_ptr<DomainState>> PDDLDomain::get_successor_states(const DomainState& curr_state) {
    vector<shared_ptr<DomainState>> successor_states;

    // Cast current DomainState to PDDLState
    const auto currPddlStatePtr = dynamic_cast<const PDDLState*>(&curr_state);
    if (!currPddlStatePtr) {
        cerr << "ERROR: the cast to PDDLState failed!" << endl;
        return successor_states; 
    }

    // Get a "raw" pddlboat::StatePtr
    auto pddlboatStatePtr = currPddlStatePtr->getPddlboatStatePtr();

    // Iterate over all non-grounded actions in the domain
    for (const auto& action : actions_) {

        // Get a "raw" pddlboat::ActionPtr
        auto pddlboatActionPtr = action->getPddlboatActionPtr();
        // Get a list of action parameters.
        auto actionParams = pddlboatActionPtr->parameters;

        // Iterate over all possible groundings for this action.
        for (const auto& assignment : pddlProblem_->getGroundings(actionParams)) {
            // Check if the action's preconditions hold in the current state
            if (action->checkPreconditions(pddlboatStatePtr, assignment)) {
                // Clone the current state to apply the effect
                auto newStatePtr = pddlboatStatePtr->clone();

                // Apply the action effect
                if (action->applyEffect(newStatePtr, pddlboatStatePtr, assignment)) {
                    // Add the new state to the successor states
                    successor_states.push_back(make_shared<PDDLState>(newStatePtr));
                }
            }
        }
    }

    return successor_states;
}

const vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> PDDLDomain::get_successor_state_action_pairs(const DomainState& curr_state) {
    vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> state_action_pairs;

    // Cast current DomainState to PDDLState
    const auto currPddlStatePtr = dynamic_cast<const PDDLState*>(&curr_state);
    if (!currPddlStatePtr) {
        cerr << "ERROR: the cast to PDDLState failed!" << endl;
        return state_action_pairs; 
    }

    // Get a "raw" pddlboat::StatePtr
    auto pddlboatStatePtr = currPddlStatePtr->getPddlboatStatePtr();

    // Iterate over all non-grounded actions in the domain
    for (const auto& action : actions_) {

        // Get a "raw" pddlboat::ActionPtr
        auto pddlboatActionPtr = action->getPddlboatActionPtr();
        // Get a list of action parameters.
        auto actionParams = pddlboatActionPtr->parameters;

        // Iterate over all possible groundings for this action.
        for (const auto& assignment : pddlProblem_->getGroundings(actionParams)) {
            // Check if the action's preconditions hold in the current state
            if (action->checkPreconditions(pddlboatStatePtr, assignment)) {
                // Clone the current state to apply the effect
                auto newStatePtr = pddlboatStatePtr->clone();

                // Apply the action effect
                if (action->applyEffect(newStatePtr, pddlboatStatePtr, assignment)) {

                    // Create a new PDDLState with the resulting state
                    auto newState = make_shared<PDDLState>(newStatePtr);
                    // Ground the action: returns a pointer to a new PDDLAction.
                    auto groundedAction = action->ground(assignment);
                    // Add the new state to the successor states
                    state_action_pairs.emplace_back(newState, groundedAction);
                }
            }
        }
    }

    return state_action_pairs;
}


bool PDDLDomain::is_valid_state(const DomainState& state) const {
    // Placeholder logic
    return true;
}

// Placeholder for mark_as_explored function
void PDDLDomain::mark_as_explored(const DomainState& state) {
    // do nothing
}

// Placeholder for mark_all_states_as_unexplored function
void PDDLDomain::mark_all_states_as_unexplored() {
    // do nothing
}

std::shared_ptr<pddlboat::Domain> PDDLDomain::getPddlboatDomainPtr() const {
    return pddlDomain_;
}

void PDDLDomain::setProblem(pddlboat::ProblemPtr problemPtr) {
    pddlProblem_ = problemPtr;
}