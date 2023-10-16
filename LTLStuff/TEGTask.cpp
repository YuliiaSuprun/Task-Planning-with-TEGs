#include "TEGTask.h"

#include <stdexcept>
#include <fstream>
#include <sstream>
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
    // Get a name for output files.
    stringstream ss;
    ss << "dfa" << task_id_;
    filename_ = ss.str();

    // Create a bdd dict.
    bdd_dict_ = make_shared<spot::bdd_dict>();

    // Register all the propositions you need here.
    for (const auto& prop_pair : formula_.get_ap_mapping()) {
        string prop = prop_pair.first;
        spot::formula prop_formula = spot::formula::ap(prop);
        bdd_dict_->register_proposition(prop_formula, nullptr);
    }
    // Print the "proposition to bdd" mapping.
    // bdd_dict_->dump(std::cout) << "---\n";

    // Compute the DFA corresponding to the given LTL formula.
    dfa_ = convert_to_dfa(formula_);
    save_dfa(dfa_);
    // Compute the product graph of DFA and GridWorldDomain.
    compute_product();
}

TEGTask::~TEGTask() {
    // Unregister all propositions: need for memory management.
    bdd_dict_->unregister_all_my_variables(nullptr);
    // When the reference count drops to zero, the destructor for the spot::bdd_dict will be triggered.
}


shared_ptr<spot::twa_graph> TEGTask::convert_to_dfa(const LTLFormula& formula) {
    spot::formula spot_formula = formula.get_spot_formula();
    if (spot_formula == nullptr) {
        cerr << "Failed to parse the LTL formula" << endl;
        return nullptr;
    }
    // Create a translator instance. 
    spot::translator trans(bdd_dict_);

    // No guarantee that the automaton will be "Deterministic" 
    // if we use TGBA or BA (Büchi Automaton) options in set_type() 
    // We must use "Generic" option if we absolutely want determinism!
    trans.set_type(spot::postprocessor::Generic);
    // Small and Deterministic are exclusive choices for set_pref
    // indicate whether a smaller non-deterministic automaton should be preferred over a deterministic automaton
    trans.set_pref(spot::postprocessor::Deterministic); 
    // Translate LTL formula to a DFA using Spot tranlsator.
    auto translated_dfa = trans.run(spot_formula);
    return translated_dfa;
}

void TEGTask::save_dfa(const shared_ptr<spot::twa_graph>& dfa) {

    string dot_filename = filename_ + ".dot";
    string png_filename = filename_ + ".png";

    ofstream out(dot_filename);
    spot::print_dot(out, dfa);
    out.close();

    // Render the DFA dot to an image.
    GVC_t* gvc = gvContext();
    FILE* file = fopen(dot_filename.c_str(), "r");
    if (!file) {
        cerr << "Failed to open " << dot_filename << " for reading." << endl;
        return;
    }

    Agraph_t* graph = agread(file, 0);
    gvLayout(gvc, graph, "dot");
    gvRenderFilename(gvc, graph, "png", png_filename.c_str());
    gvFreeLayout(gvc, graph);
    agclose(graph);
    gvFreeContext(gvc);

    fclose(file);
}

void TEGTask::compute_product() {
    // Clear previous data.
    product_states_.clear();
    product_transitions_.clear();

    // Iterate over all grid world states and DFA states to compute product states.
    for (size_t r = 0; r < grid_domain_.R(); ++r) {
        for (size_t c = 0; c < grid_domain_.C(); ++c) {
            GridState grid_state(r, c);
            // States in dfa are always numbered from 0 to (num_states-1)
            for (size_t dfa_state = 0; dfa_state < dfa_->num_states(); ++dfa_state) {
                ProductState ps(grid_state, dfa_state);
                product_states_.push_back(ps);
            }
        }
    }
    cout << product_states_.size() << " product states were generated" << endl;

    // Compute transitions in the product based on valid transitions in both the GridWorld and the DFA.
    for (const auto& prod_state : product_states_) {
        GridState grid_state = prod_state.get_grid_state();
        size_t dfa_state = prod_state.get_dfa_state();
        
        for (const auto& action : grid_domain_.get_actions()) {
            GridState next_grid_state = grid_state.apply(action);
            if (!grid_domain_.is_valid_state(next_grid_state)) {
                continue;
            }
            // Get the atomic propositions at the next grid world state.
            set<string> next_grid_state_props = atomic_props(next_grid_state);
            set<string> curr_grid_state_props = atomic_props(grid_state);
            if (curr_grid_state_props == next_grid_state_props) {
                // DFA state stays the same.
                // Any grid transition would be valid.
                product_transitions_[prod_state].push_back(ProductState(next_grid_state, dfa_state));
            }
            next_grid_state_props.insert(curr_grid_state_props.begin(), curr_grid_state_props.end());
            // BDD operations
            bdd bdd_expr = props_to_bdd(next_grid_state_props);
            // cout << "Resulting bdd_expr: " << bdd_expr << endl;
            for (auto& edge: dfa_->out(dfa_state)) {
                // cout << "DFA edge condition: " << edge.cond << endl;
                if ((edge.cond & bdd_expr) == bdd_expr) {
                    // cout << "Transition is added!!" << endl;
                    size_t next_dfa_state = edge.dst;
                    product_transitions_[prod_state].push_back(ProductState(next_grid_state, next_dfa_state));
                    break;
                }
            }
            // For garbage collection purposes.
            bdd_expr = bddfalse;
        }
    }

    // print_product_transitions(2, 1);
}

set<string> TEGTask::atomic_props(const GridState& grid_state) {
    set<string> props;

    // Iterate over the ap_mapping to check which atomic propositions hold true for the grid_state
    for (const auto& prop_pair : formula_.get_ap_mapping()) {
        const std::string& ap = prop_pair.first;
        const std::set<GridState>& ap_states = prop_pair.second;

        if (ap_states.find(grid_state) != ap_states.end()) {
            props.insert(ap);
        }
    }

    return props;
}

/*
    Return the BDD representation (from the BDD package) of the logical
    conjunction of the atomic propositions (positive and negative).
*/
bdd TEGTask::props_to_bdd(const set<string>& props) {
    // Start with a BDD representing true
    bdd result = bddtrue;

    for (const auto& prop_pair : formula_.get_ap_mapping()) {
        string prop = prop_pair.first;
        spot::formula prop_formula = spot::formula::ap(prop);
        int var_num = bdd_dict_->varnum(prop_formula);
        if (var_num == -1) {
            cerr << "ERROR: proposition is not registered!" << endl;
        }
        bdd prop_bdd = bdd_ithvar(var_num);

        if (props.find(prop) == props.end()) {
            // Proposition is absent (negative form)
            prop_bdd = bdd_not(prop_bdd);
        }

        // Logical AND with the result
        result &= prop_bdd;
    }

    return result;
}



vector<ProductState> TEGTask::solve() {
    product_path_.clear();

    // The BFS queue.
    deque<ProductState> queue;
    queue.emplace_back(start_grid_state_, static_cast<size_t>(dfa_->get_init_state_number()));

    // Keep track of visited states.
    set<ProductState> visited;
    visited.insert(queue.front());

    // To backtrack the path.
    map<ProductState, ProductState> parent_map;

    while (!queue.empty()) {
        ProductState current_state = queue.front();
        queue.pop_front();

        // Check if the DFA part of the product state is accepting.
        if (dfa_->state_is_accepting(current_state.get_dfa_state())) {
            product_path_.push_back(current_state);
            
            // Backtrack to get the full path.
            while (parent_map.find(current_state) != parent_map.end()) {
                current_state = parent_map.at(current_state);
                product_path_.push_back(current_state);
            }
            reverse(product_path_.begin(), product_path_.end());
            save_paths();
            return product_path_;
        }

        // Explore successors.
        for (const auto& next_state : product_transitions_[current_state]) {
            if (visited.find(next_state) == visited.end()) {
                queue.push_back(next_state);
                visited.insert(next_state);
                parent_map.emplace(next_state, current_state);
            }
        }
    }

    // No solution found.
    return vector<ProductState>();
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

vector<size_t> TEGTask::get_dfa_path() const {
    return dfa_path_;
}

void TEGTask::print_product_transitions(int in_dfa_state, int out_dfa_state) {
    cout << "Product Transitions:" << endl;
    for (const auto& transition_entry : product_transitions_) {
        const ProductState& source_state = transition_entry.first;
        if (in_dfa_state == -1 || static_cast<size_t>(in_dfa_state) == source_state.get_dfa_state()) {
            cout << source_state << " -> "; 
            const auto& target_states = transition_entry.second;
            for (const auto& target_state : target_states) {
                if (out_dfa_state == -1 || static_cast<size_t>(out_dfa_state) == target_state.get_dfa_state()) {
                    cout << target_state << ", ";
                }
            }
            cout << endl;
        }
    }
}

void TEGTask::print_product_path() const {
    cout << "Product Path:" << endl;
    if (!product_path_.empty()) {
        cout << product_path_.front();
        for (auto it = next(product_path_.begin()); it != product_path_.end(); ++it) {
            cout << " -> " << *it;
        }
    }
    cout << endl;
}

void TEGTask::print_grid_path() const {
    cout << "Grid Path:" << endl;
    if (!grid_path_.empty()) {
        cout << grid_path_.front();
        for (auto it = next(grid_path_.begin()); it != grid_path_.end(); ++it) {
            cout << " -> " << *it;
        }
    }
    cout << endl;
}

void TEGTask::print_dfa_path() const {
    cout << "DFA Path:" << endl;
    if (!dfa_path_.empty()) {
        cout << dfa_path_.front();
        for (auto it = next(dfa_path_.begin()); it != dfa_path_.end(); ++it) {
            cout << " -> " << *it;
        }
    }
    cout << endl;
}

map<string, set<GridState>> TEGTask::get_ap_mapping() const {
    return formula_.get_ap_mapping();
}

string TEGTask::get_filename() const {
    return filename_;
}

