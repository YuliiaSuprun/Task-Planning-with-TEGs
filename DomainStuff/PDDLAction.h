#ifndef PDDL_ACTION_H
#define PDDL_ACTION_H

#include "PrimitiveAction.h"
#include <pddlboat/solver/domain.hpp>
#include <memory> 

class PDDLAction : public PrimitiveAction {
public:
    // Constructor for ungrounded action
    PDDLAction(const pddlboat::ActionPtr& action);

    // Constructor for grounded action
    PDDLAction(const pddlboat::ActionPtr& action, const pddlboat::Assignment& assignment);


    // Override the PrimitiveAction methods
    virtual ~PDDLAction() = default;
    bool operator==(const PrimitiveAction& other) const override;
    bool operator<(const PrimitiveAction& other) const override;

    // Additional methods to interact with the pddlboat::Action
    std::string getName() const;

    /** \brief Get the arity of this action, i.e., the number of parameters.
     *  \return The arity of the action.
     */
    unsigned int getArity() const;

    /** \brief Checks if an action's preconditions hold at a world state and a grounding of the action.
     *  \param[in] state State of the problem.
     *  \param[in] vars Grounding of the action.
     *  \return True if the preconditions of this grounded action hold.
     */
    bool checkPreconditions(const pddlboat::StatePtr& state, const pddlboat::Assignment& vars) const;

    /** \brief Apply the effects of this action grounded to an assignment to a state.
     *  \param[out] out State that will have the effects of this action applied to \a in.
     *  \param[in] in Starting state. Preconditions must hold.
     *  \param[in] vars Grounding of this action.
     *  \return True on success, false on failure.
     */
    bool applyEffect(pddlboat::StatePtr& out, const pddlboat::StatePtr& in, const pddlboat::Assignment& vars) const;

    // Method to get the wrapped pddlboat::Action
    const pddlboat::ActionPtr& getPddlboatActionPtr() const;

    std::shared_ptr<PDDLAction> ground(const pddlboat::Assignment& assignment) const;
    bool isGrounded() const;

private:
    pddlboat::ActionPtr pddlboatAction_;
    pddlboat::Assignment assignment_;
    bool grounded_;
};

#endif // PDDL_ACTION_H
