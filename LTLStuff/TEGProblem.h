#ifndef TEG_TASK_H
#define TEG_TASK_H

#include "GridWorldDomain.h"
#include "GridState.h"
#include "ProductState.h"
#include "ProductTransition.h"
#include "LTLFormula.h"
#include "DFANode.h"

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
#include <string>
#include <set>

using namespace std;

class TEGProblem {
public:
    TEGProblem(const string formula_str,
            const map<string, set<GridState>> ap_mapping,
            const GridWorldDomain& grid_domain,
            const GridState& start_grid_state,
            int problem_id = 0, bool on_the_fly=false);

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
    bdd props_to_bdd(const set<string>& props);
    shared_ptr<spot::twa_graph> convert_to_dfa(const LTLFormula& formula);
    void save_dfa(const shared_ptr<spot::twa_graph>& dfa);
    void compute_product();
    void solve_with_full_graph();
    void solve_with_on_the_fly_graph();
    void save_paths();
    void print_dfa();
    void print_product_transitions(int in_dfa_state=-1, int out_dfa_state=-1);
    bdd get_self_edge_cond(size_t dfa_state);

    vector<size_t> generate_dfa_path();
    void generate_successors(const ProductState& prod_state);
    void realize_dfa_trace(vector<size_t>& dfa_trace);
    spot::twa_graph::edge_storage_t* find_transition(const GridState& next_grid_state, const GridState& curr_grid_state, size_t curr_dfa_state);

    bool is_transition_valid(const bdd& edge_cond, const bdd& next_state_bdd);


    // Class members
    LTLFormula formula_;
    GridWorldDomain grid_domain_;
    GridState start_grid_state_;
    int problem_id_;
    bool on_the_fly_;

    // DFA corresponding to LTL formula.
    shared_ptr<spot::twa_graph> dfa_;

    // Nodes and edges in the product graph.
    vector<ProductState> full_product_states_;
    map<ProductState, vector<ProductTransition>> full_product_transitions_;

    deque<shared_ptr<DFANode>> nodeQueue_;

    // Solution path (if found).
    vector<ProductState> product_path_;
    vector<GridState> grid_path_;
    vector<size_t> dfa_path_;

    string filename_;
    spot::bdd_dict_ptr bdd_dict_;
};

#endif // TEG_TASK_H
