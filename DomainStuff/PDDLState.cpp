#include "PDDLState.h"
#include "PDDLAction.h"

PDDLState::PDDLState(const pddlboat::StatePtr& state) 
    : pddlboatState_(state), isCached_(false) {}

void PDDLState::cache() {
    isCached_ = true;
}

bool PDDLState::isCached() const {
    return isCached_;
}

// std::shared_ptr<DomainState> PDDLState::apply(const PrimitiveAction& action,  const pddlboat::Assignment& vars) const {
//     try {
//         // Cast the PrimitiveAction to PDDLAction
//         const auto& pddlAction = dynamic_cast<const PDDLAction&>(action);

//         // Create a new state to hold the result of applying the action
//         pddlboat::StatePtr newState = pddlboatState_->clone();

//         // Apply the action's effects to the new state
//         if (pddlAction.applyEffect(newState, pddlboatState_, vars)) {
//             // Return the new PDDLState if the action is successfully applied
//             return std::make_shared<PDDLState>(newState);
//         }

//         // Return nullptr if the action cannot be applied
//         return nullptr;

//     } catch (const std::bad_cast&) {
//         // Handle the case where the cast fails
//         return nullptr;
//     }
// }


double PDDLState::distance(const DomainState& other) const {
    // Implement a distance metric between states if needed
    return 0.0; // Placeholder
}

bool PDDLState::operator==(const DomainState& other) const {
    try {
        const auto& otherPddlState = toPDDLState(other);
        return pddlboatState_->isEqual(otherPddlState.pddlboatState_);
    } catch (const std::bad_cast&) {
        return false;
    }
}

bool PDDLState::operator<(const DomainState& other) const {
    try {
        const auto& otherPddlState = toPDDLState(other);
        // Compare by hash.
        return pddlboatState_->hash() < otherPddlState.pddlboatState_->hash();
    } catch (const std::bad_cast&) {
        return false;
    }
}


std::ostream& PDDLState::print(std::ostream& os) const {
    // Print the state
    return pddlboatState_->toPDDL(os);
}

const PDDLState& PDDLState::toPDDLState(const DomainState& state) const {
    return dynamic_cast<const PDDLState&>(state);
}

pddlboat::StatePtr PDDLState::getPddlboatStatePtr() const {
    return pddlboatState_;
}
