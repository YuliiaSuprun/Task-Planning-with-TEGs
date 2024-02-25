#include "DomainManager.h"

using namespace std;

DomainManager::DomainManager(shared_ptr<spot::bdd_dict> bddDict, shared_ptr<GridWorldDomain> domain, const map<string, set<GridState>> ap_mapping)
: bdd_dict_(bddDict), domain_(domain), ap_mapping_(ap_mapping) {
    // Register all the propositions we need to bdd_dict.
    for (const auto& prop_pair : ap_mapping_) {
        string prop = prop_pair.first;
        spot::formula prop_formula = spot::formula::ap(prop);
        bdd_dict_->register_proposition(prop_formula, nullptr);
    }
    // Print the "proposition to bdd" mapping.
    // bdd_dict_->dump(std::std::cout) << "---\n";

    // Initialize the BDD-to-state mapping and state-to-BDD mapping
    for (const auto& domain_state : get_all_domain_states()) {
        bdd domain_state_bdd = generate_bdd(domain_state);

        // Add the mappings
        bdd_to_states_[domain_state_bdd].insert(domain_state);
        state_to_bdd_[domain_state] = domain_state_bdd;
    }
}

const vector<GridState>& DomainManager::get_all_domain_states() {
    return domain_->get_all_states();
}

const vector<pair<GridState, GridAction>> DomainManager::get_successor_state_action_pairs(GridState& domain_state) {
    return domain_->get_successor_state_action_pairs(domain_state);
}

/*
    Return the BDD representation (from the BDD package) of the logical
    conjunction of the atomic propositions (positive and negative).
*/
bdd DomainManager::generate_bdd(const GridState& domain_state) {

    // Start with a BDD representing true
    bdd result = bddtrue;

    // Iterate over the ap_mapping to determine whether each ap is + or -
    for (const auto& prop_pair : ap_mapping_) {
        string prop = prop_pair.first;
        spot::formula prop_formula = spot::formula::ap(prop);
        int var_num = bdd_dict_->varnum(prop_formula);
        if (var_num == -1) {
            cerr << "ERROR: proposition is not registered!" << endl;
        }
        bdd prop_bdd = bdd_ithvar(var_num);

        auto ap_states = prop_pair.second;

        if (ap_states.find(domain_state) == ap_states.end()) {
            // prop is absent (negative form)
            prop_bdd = bdd_not(prop_bdd);
        }   // Otherwise, prop has a positive form. Do nothing.

        // Logical AND with the result
        result &= prop_bdd;
    }

    return result;
}

/*
    Return the BDD representation (from the BDD package) of the logical
    conjunction of the atomic propositions (positive and negative).
*/
bdd DomainManager::get_state_bdd(const GridState& domain_state) {
    return state_to_bdd_[domain_state];
}

set<GridState>& DomainManager::get_equivalence_region(bdd& equiv_region_bdd) {
    return bdd_to_states_[equiv_region_bdd];
}

set<string> DomainManager::atomic_props(const GridState& domain_state) {
    set<string> props;

    // Iterate over the ap_mapping to check which atomic propositions hold true for the grid_state
    for (const auto& prop_pair : ap_mapping_) {
        const std::string& ap = prop_pair.first;
        const std::set<GridState>& ap_states = prop_pair.second;

        if (ap_states.find(domain_state) != ap_states.end()) {
            props.insert(ap);
        }
    }

    return props;
}