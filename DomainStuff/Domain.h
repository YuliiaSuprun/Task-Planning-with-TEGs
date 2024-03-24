#ifndef DOMAIN_H
#define DOMAIN_H

#include <vector>
#include <set>
#include <map>

#include "DomainState.h"
#include "PrimitiveAction.h"

class Domain {
public:
    virtual ~Domain() = default;

    // Pure virtual functions that must be implemented by derived classes.
    // virtual const std::vector<std::shared_ptr<PrimitiveAction>>& get_actions(const DomainState& state) const = 0;
    virtual const std::vector<std::shared_ptr<DomainState>>& get_all_states() = 0;
    virtual const std::vector<std::shared_ptr<DomainState>> get_successor_states(const DomainState& curr_state) = 0; 
    virtual const std::vector<std::pair<std::shared_ptr<DomainState>, std::shared_ptr<PrimitiveAction>>> get_successor_state_action_pairs(const DomainState& state) = 0;
    virtual bool is_valid_state(const DomainState& state) const = 0;
    virtual void mark_as_explored(const DomainState& state) = 0;
    virtual void mark_all_states_as_unexplored() = 0;
};

#endif // DOMAIN_H
