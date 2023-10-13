#include "TEGTask.h"

#include <stdexcept>
#include <fstream>
#include <iostream>
#include <graphviz/gvc.h>

TEGTask::TEGTask(const LTLFormula& formula,
                const GridWorldDomain& grid_domain,  
                const GridState& start_grid_state, int task_id)
    : formula_(formula), grid_domain_(grid_domain), 
      start_grid_state_(start_grid_state), task_id_(task_id) { 
    // Check if the start state is valid.
    if (!grid_domain_.is_valid_state(start_grid_state_)) {
        cerr << "ERROR: Invalid start state in the GridWorldDomain" << endl;
    }
    cout << "Creating dfa for task_id=" << task_id_ << endl;
    // Compute the DFA corresponding to the given LTL formula.
    dfa_ = convert_to_dfa(formula_);
    save_dfa(dfa_);
    // Compute the product graph of DFA and GridWorldDomain.
    compute_product();
}

shared_ptr<spot::twa_graph> TEGTask::convert_to_dfa(const LTLFormula& formula) {
    // Translate LTL formula to DFA using Spot
    spot::formula spot_formula = formula.get_spot_formula();
    if (spot_formula == nullptr) {
        cerr << "Failed to parse the LTL formula" << endl;
        return nullptr;
    }
    // auto translated_dfa = spot::translate(spot_formula, spot::postprocessor::HOA);
    // return translated_dfa;
    return nullptr;
}

void TEGTask::save_dfa(const shared_ptr<spot::twa_graph>& dfa) {
    ofstream out("dfa.dot");
    spot::print_dot(out, dfa);
    out.close();

    // Render the DFA dot to an image (this assumes you have Graphviz installed)
    GVC_t* gvc = gvContext();
    Agraph_t* graph = agread(new ifstream("dfa.dot"), 0);
    gvLayout(gvc, graph, "dot");
    gvRenderFilename(gvc, graph, "png", "dfa.png");
    gvFreeLayout(gvc, graph);
    agclose(graph);
    gvFreeContext(gvc);
}

void TEGTask::compute_product() {
    // Here you'd implement the product computation logic.
    // It's quite involved, so you'll need to define your transition functions, atomic proposition mappings, etc.

    // Compute values for vector<ProductState> product_states_ and map<ProductState, vector<ProductState>> product_transitions_ here.
}

set<string> TEGTask::atomic_props(const GridState& grid_state) {
    (void)grid_state;
    set<string> props;
    // Map the grid state to its atomic propositions here.
    // You'll need to have some data structure to know which propositions are valid at each state.
    return props;
}

vector<ProductState> TEGTask::solve() {
    product_path_.clear();
    // Implement the search/solution logic here.
    // Update product_path_ 
    save_paths();
    return product_path_;
}

void TEGTask::save_paths() {
    grid_path_.clear();
    dfa_path_.clear();
    int prev_dfa_state = -1;
    for(const auto& ps : product_path_) {
        grid_path_.push_back(ps.get_grid_state());
        int curr_dfa_state = ps.get_dfa_state();
        if (prev_dfa_state != curr_dfa_state) {
            dfa_path_.push_back(curr_dfa_state);
            prev_dfa_state = curr_dfa_state;
        }
    }
}

vector<GridState> TEGTask::get_grid_path() const {
    return grid_path_;
}

vector<int> TEGTask::get_dfa_path() const {
    return dfa_path_;
}
