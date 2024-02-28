#ifndef DOMAIN_STATE_H
#define DOMAIN_STATE_H

#include <iostream>
#include <set>

#include "GridAction.h"

class DomainState {
public:
    virtual ~DomainState() = default;

    // Virtual functions to be overridden by derived classes
    virtual void cache() = 0;
    virtual bool isCached() const = 0;
    virtual std::shared_ptr<DomainState> apply(const GridAction& action) const = 0;
    virtual double distance(const DomainState& other) const = 0;

    // Operators to be overridden
    virtual bool operator==(const DomainState& other) const = 0;
    virtual bool operator<(const DomainState& other) const = 0;

    friend std::ostream& operator<<(std::ostream& os, const DomainState& state) {
        return state.print(os);
    }

protected:
    virtual std::ostream& print(std::ostream& os) const = 0;
};

struct DomainStatePtrComparator {
    bool operator()(const std::shared_ptr<DomainState>& lhs, const std::shared_ptr<DomainState>& rhs) const {
        // Assuming DomainState has an appropriate comparison method
        // This will compare the objects pointed to, not the pointers themselves
        return *lhs < *rhs; 
    }
};

using DomainStateSet = std::set<std::shared_ptr<DomainState>, DomainStatePtrComparator>;

#endif // DOMAIN_STATE_H
