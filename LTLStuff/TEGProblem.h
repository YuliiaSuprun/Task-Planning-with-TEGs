#ifndef TEG_TASK_H
#define TEG_TASK_H

#include "GridWorldDomain.h"
#include "GridState.h"
#include "ProductState.h"
#include "ProductTransition.h"
#include "LTLFormula.h"
#include "DFANode.h"
#include "DFAManager.h"
#include "Constants.h"

#include <spot/tl/formula.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/totgba.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/remprop.hh>
#include <spot/twaalgos/postproc.hh> 
#include <spot/twa/bddprint.hh>
#include <spot/tl/ltlf.hh>

#include <bddx.h>
#include <map>
#include <vector>
#include <deque>
#include <queue>
#include <unordered_map>
#include <string>
#include <set>

using namespace std;

class TEGProblem {
public:
    TEGProblem(const string formula_str,
            const map<string, set<GridState>> ap_mapping,
            const GridWorldDomain& grid_domain,
            const GridState& start_grid_state,
            int problem_id = 0, bool on_the_fly=false, bool cache=false, bool feedback=false);

    ~TEGProblem();
    
    vector<ProductState> solve();
    vector<GridState> get_grid_path() const;
    vector<size_t> get_dfa_path() const;
    map<string, set<GridState>> get_ap_mapping() const;
    string get_filename() const;

    void print_product_path() const;
    void print_grid_path() const;
    void print_dfa_path() const;
    void print_dfa_path(vector<size_t> dfa_path) const;

private:
    set<string> atomic_props(const GridState& grid_state);
    bdd get_state_bdd(const GridState& grid_state);
    void compute_product();
    void solve_with_full_graph();
    void solve_with_on_the_fly_graph();
    void save_paths();
    void print_product_transitions(int in_dfa_state=-1, int out_dfa_state=-1);

    shared_ptr<DFANode> generate_dfa_path();
    void generate_successors(const ProductState& prod_state);
    void realize_dfa_trace(shared_ptr<DFANode>& endTraceNode);
    spot::twa_graph::edge_storage_t* find_transition(const GridState& next_grid_state, size_t curr_dfa_state);

    bool is_transition_valid(const bdd& edge_cond, const bdd& next_state_bdd);
    vector<ProductState> construct_path(const map<ProductState, vector<ProductState>>& parent_map, ProductState target_state, bool cached=false, size_t start_dfa_state=0);


    int dfa_transition_cost(size_t from_state, size_t to_state);
    void update_dfa_transition_cost(shared_ptr<DFANode>& node, int cost);

    void print_node_priority_queue();


    // Class members
    LTLFormula formula_;
    GridWorldDomain grid_domain_;
    GridState start_grid_state_;
    int problem_id_;
    bool on_the_fly_;
    bool cache_;
    bool feedback_;

    // DFA corresponding to LTL formula.
    // shared_ptr<spot::twa_graph> dfa_;
    spot::bdd_dict_ptr bdd_dict_;
    DFAManager dfa_manager_;

    // Nodes and edges in the product graph.
    vector<ProductState> full_product_states_;
    map<ProductState, vector<ProductTransition>> full_product_transitions_;

    // A priority queue with a pair of (cost, DFANode)
    NodeHeap nodePriorityQueue_;

    map<shared_ptr<DFANode>, NodeHeap::handle_type> node_handles_;


    // Map to store the cost of transitions
    map<pair<size_t, size_t>, int> dfa_transition_costs_;


    // Solution path (if found).
    vector<ProductState> product_path_;
    vector<GridState> grid_path_;
    vector<size_t> dfa_path_;

    string filename_;
};

#endif // TEG_TASK_H
