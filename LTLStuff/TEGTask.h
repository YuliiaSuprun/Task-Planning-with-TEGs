#ifndef TEG_TASK_H
#define TEG_TASK_H

#include "GridWorldDomain.h"
#include "GridState.h"
#include "ProductState.h"
#include "LTLFormula.h"
#include <spot/tl/formula.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twa/bddprint.hh>
#include <bddx.h>
#include <map>
#include <vector>
#include <string>
#include <set>

using namespace std;

class TEGTask {
public:
    TEGTask(const LTLFormula& formula, const GridWorldDomain& grid_domain,  
             const GridState& start_grid_state, int task_id = 0);

    ~TEGTask();
    
    vector<ProductState> solve();
    vector<GridState> get_grid_path() const;
    vector<int> get_dfa_path() const;

private:
    set<string> atomic_props(const GridState& grid_state);
    bdd props_to_bdd(const set<string>& props);
    shared_ptr<spot::twa_graph> convert_to_dfa(const LTLFormula& formula);
    void save_dfa(const shared_ptr<spot::twa_graph>& dfa);
    void compute_product();
    void save_paths();

    // Class members
    LTLFormula formula_;
    GridWorldDomain grid_domain_;
    GridState start_grid_state_;
    int task_id_;

    // DFA corresponding to LTL formula.
    shared_ptr<spot::twa_graph> dfa_;

    // Nodes and edges in the product graph.
    vector<ProductState> product_states_;
    map<ProductState, vector<ProductState>> product_transitions_;

    // Solution path (if found).
    vector<ProductState> product_path_;
    vector<GridState> grid_path_;
    vector<int> dfa_path_;

    string filename_;
    spot::bdd_dict_ptr bdd_dict_;
};

#endif // TEG_TASK_H
