#ifndef PRODUCTMANAGER_H
#define PRODUCTMANAGER_H

#include <memory>
#include <vector>
#include <map>
#include <set>
#include <random>
#include "ProductState.h"
#include "ProductTransition.h"
#include "GridWorldDomain.h"
#include "DFAManager.h"
#include "DomainManager.h"

class ProductManager {
public:
    ProductManager(std::shared_ptr<DomainManager> domain_manager, 
                   std::shared_ptr<DFAManager> dfa_manager);

    void generate_all_product_states();
    void compute_full_product();
    void generate_successors(const ProductState& prod_state);

    std::vector<ProductTransition> get_transitions(const ProductState& prod_state);

    bool state_not_expanded(const ProductState& prod_state);
    void cache_path(vector<ProductState>& path_to_cache_reversed, bdd edge_condition);

    void print_product_transitions(int in_dfa_state=-1, int out_dfa_state=-1);

    set<GridState> sample_landmarks(const bdd& edge_cond, const GridState& curr_domain_state, size_t num_landmarks=NUM_LANDMARKS);

private:
    std::shared_ptr<DomainManager> domain_manager_;
    std::shared_ptr<DFAManager> dfa_manager_;
    std::vector<ProductState> product_states_;
    std::map<ProductState, std::vector<ProductTransition>> product_transitions_;
    static const size_t NUM_LANDMARKS = 100;
};

#endif // PRODUCTMANAGER_H
