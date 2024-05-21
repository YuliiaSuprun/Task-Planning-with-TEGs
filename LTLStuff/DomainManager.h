#ifndef DOMAINMANAGER_H
#define DOMAINMANAGER_H

#include <memory>
#include <spot/twa/bddprint.hh>
#include "Domain.h" 

struct BddComparator {
    bool operator()(const bdd& lhs, const bdd& rhs) const {
        return lhs.id() < rhs.id();
    }
};

using namespace std;

class DomainManager {
public:
    DomainManager(shared_ptr<spot::bdd_dict> bddDict, shared_ptr<Domain> domain, const map<string, DomainStateSet> ap_to_states_mapping);
    DomainManager(shared_ptr<spot::bdd_dict> bddDict, shared_ptr<Domain> domain, const map<string, pair<string, vector<string>>> pred_mapping);

    const vector<shared_ptr<DomainState>>& get_all_domain_states();

    const vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> get_successor_state_action_pairs(const DomainState& domain_state);

    bdd get_state_bdd(const shared_ptr<DomainState> domain_state);
    DomainStateSet get_equivalence_region(bdd& equiv_region_bdd);
    set<bdd, BddComparator> get_all_equivalence_regions();
    map<bdd, DomainStateSet, BddComparator>& get_bdd_to_states_map();
    set<string> atomic_props(const shared_ptr<DomainState> domain_state);
    string get_ap_name(const int var_num);

    void print_ap_to_states_mapping();

private:

    bdd generate_bdd(const shared_ptr<DomainState> domain_state);

    shared_ptr<spot::bdd_dict> bdd_dict_;
    shared_ptr<Domain> domain_;

    map<string, DomainStateSet> ap_to_states_mapping_;
    map<string, pair<string, vector<string>>> pred_mapping_;

    map<bdd, DomainStateSet, BddComparator> bdd_to_states_;
    map<shared_ptr<DomainState>, bdd, DomainStatePtrComparator> state_to_bdd_;
    map<int, string> bdd_to_ap_;

    bool use_pred_mapping_;
};

#endif // DOMAINMANAGER_H
