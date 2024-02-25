#include "ProductManager.h"

using namespace std;

ProductManager::ProductManager(shared_ptr<DomainManager> domain_manager, 
                               shared_ptr<DFAManager> dfa_manager)
    : domain_manager_(domain_manager), dfa_manager_(dfa_manager) {}

void ProductManager::generate_all_product_states() {
    product_states_.clear();
    // Iterate over all domain states and DFA states to compute product states.
    for (const auto& domain_state : domain_manager_->get_all_domain_states()) {
        // States in dfa are always numbered from 0 to (num_states-1)
        for (size_t dfa_state = 0; dfa_state < dfa_manager_->get_num_states(); ++dfa_state) {
            product_states_.emplace_back(domain_state, dfa_state);
        }
    }
    std::cout << product_states_.size() << " product states were generated" << endl;
}

void ProductManager::compute_full_product() {
    generate_all_product_states();

    product_transitions_.clear();

    // Compute transitions in the product based on valid transitions in both the GridWorld and the DFA.
    for (const auto& prod_state : product_states_) {
        generate_successors(prod_state);
    }
}

void ProductManager::generate_successors(const ProductState& prod_state) {
    size_t dfa_state = prod_state.get_dfa_state();
    GridState domain_state = prod_state.get_domain_state();

    // Add transitions based on "primitive" actions in the domain.
    for (const auto& state_action_pair : domain_manager_->get_successor_state_action_pairs(domain_state)) {
        GridState next_domain_state = state_action_pair.first;
        GridAction next_action = state_action_pair.second;

        bdd next_domain_state_bdd = domain_manager_->get_state_bdd(next_domain_state);
        auto dfa_transition = dfa_manager_->find_transition(next_domain_state_bdd, dfa_state);

        if (dfa_transition == nullptr) {
            continue;
        }

        size_t next_dfa_state = dfa_transition->dst;
        ProductState next_prod_state(next_domain_state, next_dfa_state);
        auto domain_action = make_shared<GridAction>(next_action);
        product_transitions_[prod_state].emplace_back(prod_state, next_prod_state, dfa_transition->cond, domain_action);
    }
}

vector<ProductTransition> ProductManager::get_transitions(const ProductState& prod_state) {
    return product_transitions_.at(prod_state);
}

bool ProductManager::state_not_expanded(const ProductState& prod_state) {
    return product_transitions_.count(prod_state) == 0;
}

void ProductManager::cache_path(vector<ProductState>& path_to_cache_reversed, bdd edge_condition) {
    // Find the source node of this transition.
    ProductState start_state = path_to_cache_reversed.back();
    ProductState end_state = path_to_cache_reversed.front();

    // Create compound action.
    auto compound_action = make_shared<CompoundAction>(path_to_cache_reversed);
    // Now we create a transition and add it to a product graph.
    // Serves as a "skip-connection".
    product_transitions_[start_state].emplace_back(start_state, end_state, edge_condition, compound_action, true);
}

void ProductManager::print_product_transitions(int in_dfa_state, int out_dfa_state) {
    std::cout << "Product Transitions:" << endl;
    for (const auto& transition_entry : product_transitions_) {
        const ProductState& source_state = transition_entry.first;
        if (in_dfa_state == -1 || static_cast<size_t>(in_dfa_state) == source_state.get_dfa_state()) {
            std::cout << source_state << " -> "; 
            const auto& transitions = transition_entry.second;
            for (const auto& transition : transitions) {
                ProductState target_state = transition.out_state();
                if (out_dfa_state == -1 || static_cast<size_t>(out_dfa_state) == target_state.get_dfa_state()) {
                    std::cout << target_state << ", ";
                }
            }
            std::cout << endl;
        }
    }
}
