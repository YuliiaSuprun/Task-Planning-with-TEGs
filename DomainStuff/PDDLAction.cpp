#include "PDDLAction.h"

PDDLAction::PDDLAction(const pddlboat::ActionPtr& action) 
    : pddlboatAction_(action), grounded_(false) {}

PDDLAction::PDDLAction(const pddlboat::ActionPtr& action, const pddlboat::Assignment& assignment)
    : pddlboatAction_(action), assignment_(assignment), grounded_(true) {}

bool PDDLAction::operator==(const PrimitiveAction& other) const {
    const auto& otherPddlAction = dynamic_cast<const PDDLAction&>(other);
    return this->pddlboatAction_->name == otherPddlAction.pddlboatAction_->name; // comparison by name
}

bool PDDLAction::operator<(const PrimitiveAction& other) const {
    const auto& otherPddlAction = dynamic_cast<const PDDLAction&>(other);
    return this->pddlboatAction_->name < otherPddlAction.pddlboatAction_->name; // ordering by name
}

std::string PDDLAction::getName() const {
    return pddlboatAction_->name;
}

// Get the arity of this action, i.e., the number of parameters.
unsigned int PDDLAction::getArity() const {
    return pddlboatAction_->arity();
}

// Checks if an action's preconditions hold at a world state and a grounding of the action.
bool PDDLAction::checkPreconditions(const pddlboat::StatePtr& state, const pddlboat::Assignment& vars) const {
    return pddlboatAction_->holds(state, vars);
}

// Apply the effects of this action grounded to an assignment to a state.
bool PDDLAction::applyEffect(pddlboat::StatePtr& out, const pddlboat::StatePtr& in, const pddlboat::Assignment& vars) const {
    return pddlboatAction_->doAction(out, in, vars);
}

const pddlboat::ActionPtr& PDDLAction::getPddlboatActionPtr() const {
    return pddlboatAction_;
}

std::shared_ptr<PDDLAction> PDDLAction::ground(const pddlboat::Assignment& assignment) const {
    // Create a grounded action.
    return std::make_shared<PDDLAction>(pddlboatAction_, assignment);
}

bool PDDLAction::isGrounded() const {
    return grounded_;
}