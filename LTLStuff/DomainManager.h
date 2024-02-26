#ifndef DOMAINMANAGER_H
#define DOMAINMANAGER_H

#include <memory>
#include <spot/twa/bddprint.hh>
#include "GridWorldDomain.h" 

struct BddComparator {
    bool operator()(const bdd& lhs, const bdd& rhs) const {
        return lhs.id() < rhs.id();
    }
};


class DomainManager {
public:
    DomainManager(shared_ptr<spot::bdd_dict> bddDict, shared_ptr<GridWorldDomain> domain, const map<string, set<GridState>> ap_mapping);

    const vector<GridState>& get_all_domain_states();

    const vector<pair<GridState, GridAction>> get_successor_state_action_pairs(GridState& domain_state);

    bdd get_state_bdd(const GridState& domain_state);
    set<GridState>& get_equivalence_region(bdd& equiv_region_bdd);
    set<bdd, BddComparator> get_all_equivalence_regions();
    map<bdd, set<GridState>, BddComparator>& get_bdd_to_states_map();
    set<string> atomic_props(const GridState& domain_state);

private:

    bdd generate_bdd(const GridState& domain_state);

    shared_ptr<spot::bdd_dict> bdd_dict_;
    shared_ptr<GridWorldDomain> domain_;
    map<string, set<GridState>> ap_mapping_;

    map<bdd, set<GridState>, BddComparator> bdd_to_states_;
    map<GridState, bdd> state_to_bdd_;
};

#endif // DOMAINMANAGER_H
