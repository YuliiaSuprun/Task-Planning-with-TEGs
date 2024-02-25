#ifndef DOMAINMANAGER_H
#define DOMAINMANAGER_H

#include <memory>
#include <spot/twa/bddprint.hh>
#include "GridWorldDomain.h" // Include your GridWorldDomain class

class DomainManager {
public:
    DomainManager(std::shared_ptr<spot::bdd_dict> bddDict, std::shared_ptr<GridWorldDomain> domain, const map<string, set<GridState>> ap_mapping);

    const vector<GridState>& get_all_domain_states();

    const vector<pair<GridState, GridAction>> get_successor_state_action_pairs(GridState& domain_state);

    bdd get_state_bdd(const GridState& domain_state);
    set<string> atomic_props(const GridState& domain_state);


private:
    std::shared_ptr<spot::bdd_dict> bdd_dict_;
    std::shared_ptr<GridWorldDomain> domain_;
    map<string, set<GridState>> ap_mapping_;
};

#endif // DOMAINMANAGER_H
