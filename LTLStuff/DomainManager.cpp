#include "DomainManager.h"
#include "PDDLState.h"

using namespace std;

DomainManager::DomainManager(shared_ptr<spot::bdd_dict> bddDict, shared_ptr<Domain> domain, const map<string, DomainStateSet> ap_to_states_mapping)
: bdd_dict_(bddDict), domain_(domain), ap_to_states_mapping_(ap_to_states_mapping), use_pred_mapping_(false) {
    // Register all the propositions we need to bdd_dict.
    for (const auto& prop_pair : ap_to_states_mapping_) {
        string prop = prop_pair.first;
        spot::formula prop_formula = spot::formula::ap(prop);
        bdd_dict_->register_proposition(prop_formula, nullptr);
    }
    // Print the "proposition to bdd" mapping.
    // bdd_dict_->dump(std::std::cout) << "---\n";

    // Initialize the BDD-to-state mapping and state-to-BDD mapping for easy domains (GridWorld)
    for (const auto& domain_state_ptr : get_all_domain_states()) {
        bdd domain_state_bdd = generate_bdd(domain_state_ptr);

        // Add the mappings
        bdd_to_states_[domain_state_bdd].insert(domain_state_ptr);
        state_to_bdd_[domain_state_ptr] = domain_state_bdd;
    }
}

DomainManager::DomainManager(shared_ptr<spot::bdd_dict> bddDict, shared_ptr<Domain> domain, const map<string, pair<string, vector<string>>> pred_mapping)
: bdd_dict_(bddDict), domain_(domain), pred_mapping_(pred_mapping), use_pred_mapping_(true) {
    // Register all the propositions we need to bdd_dict.
    for (const auto& prop_pair : pred_mapping_) {
        string prop = prop_pair.first;
        spot::formula prop_formula = spot::formula::ap(prop);
        bdd_dict_->register_proposition(prop_formula, nullptr);
    }
    // Print the "proposition to bdd" mapping.
    // bdd_dict_->dump(std::std::cout) << "---\n";
}

const vector<shared_ptr<DomainState>>& DomainManager::get_all_domain_states() {
    return domain_->get_all_states();
}

const vector<pair<shared_ptr<DomainState>, shared_ptr<PrimitiveAction>>> DomainManager::get_successor_state_action_pairs(const DomainState& domain_state) {
    return domain_->get_successor_state_action_pairs(domain_state);
}

/*
    Return the BDD representation (from the BDD package) of the logical
    conjunction of the atomic propositions (positive and negative).
*/
bdd DomainManager::generate_bdd(const shared_ptr<DomainState> domain_state) {

    // Start with a BDD representing true
    bdd result = bddtrue;

    if (use_pred_mapping_) {
        // Case 1: use the mapping from atomic props to grounded predicates
        // (PDDL problems)
        // Cast a domain state into a PDDL state!
        auto pddlStatePtr = dynamic_pointer_cast<PDDLState>(domain_state);
        if (!pddlStatePtr) {
            cerr << "ERROR: the cast to PDDLState failed!" << endl;
            return bddfalse;
        }

        auto pddlboatStatePtr = pddlStatePtr->getPddlboatStatePtr();
        // Iterate over the pred_mapping to determine whether each grounded predicate is true or false in the given domain state.
        for (const auto& prop_pair : pred_mapping_) {
            string prop = prop_pair.first;
            spot::formula prop_formula = spot::formula::ap(prop);
            int var_num = bdd_dict_->varnum(prop_formula);
            if (var_num == -1) {
                cerr << "ERROR: proposition is not registered!" << endl;
            }
            bdd prop_bdd = bdd_ithvar(var_num);

            auto grounded_predicate = prop_pair.second;
            auto predicate_name = grounded_predicate.first;
            auto assignment = grounded_predicate.second;

            if (!pddlboatStatePtr->isTrue(predicate_name, assignment)) {
                // prop is negative!
                prop_bdd = bdd_not(prop_bdd);
            }   // Otherwise, prop has a positive form. Do nothing.

            // Logical AND with the result
            result &= prop_bdd;
        }
    } else {
        // Case 2: use the mapping from atomic props to states that satisfy them
        // (simple problems, e.g. GridWorld)
        // Iterate over the ap_to_states_mapping to determine whether each ap is + or -
        for (const auto& prop_pair : ap_to_states_mapping_) {
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
    }

    return result;
}

/*
    Return the BDD representation (from the BDD package) of the logical
    conjunction of the atomic propositions (positive and negative).
*/
bdd DomainManager::get_state_bdd(const shared_ptr<DomainState> domain_state) {
    // Lazy generation.
    auto it = state_to_bdd_.find(domain_state);
    if (it != state_to_bdd_.end()) {
        // BDD already generated for this state, return it
        return it->second;
    } else {
        // Generate BDD for the new state
        bdd state_bdd = generate_bdd(domain_state);

        // Update mappings
        bdd_to_states_[state_bdd].insert(domain_state);
        state_to_bdd_[domain_state] = state_bdd;

        return state_bdd;
    }
}

DomainStateSet DomainManager::get_equivalence_region(bdd& equiv_region_bdd) {
    if (use_pred_mapping_) {
        throw std::runtime_error("get_equivalence_region() should not be called for PDDL problems!");
    }
    auto it = bdd_to_states_.find(equiv_region_bdd);
    if (it != bdd_to_states_.end()) {
        return it->second;
    } else {
        return DomainStateSet();
    }
}

set<string> DomainManager::atomic_props(const shared_ptr<DomainState> domain_state) {
    set<string> props;

    if (use_pred_mapping_) {
        // Case 1: use the mapping from atomic props to grounded predicates
        // (PDDL problems)
        // Cast a domain state into a PDDL state!
        auto pddlStatePtr = dynamic_pointer_cast<PDDLState>(domain_state);
        if (!pddlStatePtr) {
            cerr << "ERROR: the cast to PDDLState failed!" << endl;
            return props;
        }

        auto pddlboatStatePtr = pddlStatePtr->getPddlboatStatePtr();
        for (const auto& prop_pair : pred_mapping_) {
            const string& ap = prop_pair.first;
            
            auto grounded_predicate = prop_pair.second;
            auto predicate_name = grounded_predicate.first;
            auto assignment = grounded_predicate.second;

            if (pddlboatStatePtr->isTrue(predicate_name, assignment)) {
                props.insert(ap);
            }
        }
    } else {
        // Case 2: use the mapping from atomic props to states that satisfy them
        // (simple problems, e.g. GridWorld)
        // Iterate over the ap_to_states_mapping to check which atomic propositions hold true for the grid_state
        for (const auto& prop_pair : ap_to_states_mapping_) {
            const string& ap = prop_pair.first;
            const DomainStateSet& ap_states = prop_pair.second;

            if (ap_states.find(domain_state) != ap_states.end()) {
                props.insert(ap);
            }
        }
    }

    return props;
}

set<bdd, BddComparator> DomainManager::get_all_equivalence_regions() {

    if (use_pred_mapping_) {
        throw std::runtime_error("get_all_equivalence_regions() should not be called for PDDL problems!");
    }

    set<bdd, BddComparator> equivalence_regions;
    for (const auto& pair : bdd_to_states_) {
        equivalence_regions.insert(pair.first);
    }

    return equivalence_regions;
}

map<bdd, DomainStateSet, BddComparator>& DomainManager::get_bdd_to_states_map() {
    return bdd_to_states_;
}

void DomainManager::print_ap_to_states_mapping() {
    for (const auto& pair : ap_to_states_mapping_) {
        std::cout << "Key: " << pair.first << "\nStates:\n";
        for (const auto& statePtr : pair.second) {
            if (statePtr) {
                std::cout << "  " << *statePtr << "\n";
            } else {
                std::cout << "  nullptr\n";
            }
        }
        std::cout << "\n";
    }
}