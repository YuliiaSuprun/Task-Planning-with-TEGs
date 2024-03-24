#ifndef PDDL_STATE_H
#define PDDL_STATE_H

#include "DomainState.h"
#include <memory>
#include <pddlboat/solver/domain.hpp>

class PDDLState : public DomainState {
public:
    PDDLState(const pddlboat::StatePtr& state);

    // Override the DomainState methods
    virtual ~PDDLState() = default;
    void cache() override;
    bool isCached() const override;
    // std::shared_ptr<DomainState> apply(const PrimitiveAction& action,  const pddlboat::Assignment& vars) const override;
    double distance(const DomainState& other) const override;

    // Operators to be overridden
    bool operator==(const DomainState& other) const override;
    bool operator<(const DomainState& other) const override;

    pddlboat::StatePtr getPddlboatStatePtr() const;

protected:
    std::ostream& print(std::ostream& os) const override;

private:
    pddlboat::StatePtr pddlboatState_;
    bool isCached_;

    // Helper function to cast to PDDLState
    const PDDLState& toPDDLState(const DomainState& state) const;
};

#endif // PDDL_STATE_H
