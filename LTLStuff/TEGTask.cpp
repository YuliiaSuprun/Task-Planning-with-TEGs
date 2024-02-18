#include "TEGTask.h"

#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <graphviz/gvc.h>

TEGTask::TEGTask(const LTLFormula& formula,
                const GridWorldDomain& grid_domain,  
                const GridState& start_grid_state, int task_id, bool use_skills)
    : formula_(formula), grid_domain_(grid_domain), 
      start_grid_state_(start_grid_state), task_id_(task_id),
      use_skills_(use_skills) { 

    // Check if the start state is valid.
    if (!grid_domain_.is_valid_state(start_grid_state_)) {
        cerr << "ERROR: Invalid start state in the GridWorldDomain" << endl;
    }
    std::cout << "Creating dfa for task_id=" << task_id_ << endl;
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
    // bdd_dict_->dump(std::std::cout) << "---\n";

    auto start1 = chrono::high_resolution_clock::now();

    // Compute the DFA corresponding to the given LTL formula.
    dfa_ = convert_to_dfa(formula_);

    auto end1 = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_dfa = end1 - start1;
    std::cout << "time of calculating the automaton: " << elapsed_dfa.count() << " seconds" << std::endl;

    print_dfa();
    save_dfa(dfa_);
    auto start2 = chrono::high_resolution_clock::now();
    // Compute the product graph of DFA and GridWorldDomain.
    compute_product();
    auto end2 = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_product = end2 - start2;
    std::cout << "time of calculating the product: " << elapsed_product.count() << " seconds" << std::endl;
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

    // Set a type to BA (Büchi Automaton).
    trans.set_type(spot::postprocessor::Generic);
    // Small (default) and Deterministic are exclusive choices for set_pref().
    // Indicate whether a smaller non-deterministic automaton should be
    // preferred over a deterministic automaton (no guarantees tho).
    trans.set_pref(spot::postprocessor::Deterministic); 
    // The automaton must use a state-based acceptance.
    trans.set_pref(spot::postprocessor::SBAcc);
    // Translate LTL formula to a DFA using Spot tranlsator.
    spot::twa_graph_ptr translated_dfa = trans.run(spot_formula);
    
    // translated_dfa->prop_state_acc(true);
    // std::cout << "A state-based acceptance is now set to: " << translated_dfa->prop_state_acc() << endl;
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
    std::cout << product_states_.size() << " product states were generated" << endl;

    // Add transitions based on "skill" actions in the domain.
    grid_domain_.print_all_skill_actions();
    // Compute transitions in the product based on valid transitions in both the GridWorld and the DFA.
    for (const auto& prod_state : product_states_) {
        size_t dfa_state = prod_state.get_dfa_state();
        GridState grid_state = prod_state.get_grid_state();
        // This is a current label.
        set<string> curr_grid_state_props = atomic_props(grid_state);

        if (use_skills_) {
            // Use only the skill actions applicable for the current label.
            for (const auto& skill_action : grid_domain_.get_skill_actions(grid_state)) {
                // GridState intermediate_grid_state = skill_action.start_grid_state();

                // std::cout << "SkillAction is considered for computing a product." << endl;
                // skill_action.print_path();
                // std::cout << "======================" << endl;
            
                GridState next_grid_state = skill_action.dest_grid_state();
                if (!grid_domain_.is_valid_state(next_grid_state)) {
                    continue;
                }
                // Get the atomic propositions at the next grid world state.
                set<string> next_grid_state_props = atomic_props(next_grid_state);
                next_grid_state_props.insert(curr_grid_state_props.begin(), curr_grid_state_props.end());
                // BDD operations
                bdd bdd_expr = props_to_bdd(next_grid_state_props);
                for (auto& edge: dfa_->out(dfa_state)) {

                    // Could also use edge.acc, which is spot::acc_cond::mark_t
                    if ((edge.cond & bdd_expr) == bdd_expr) { 
                        size_t next_dfa_state = edge.dst;
                        ProductState next_prod_state(next_grid_state, next_dfa_state);
                        product_transitions_[prod_state].push_back(ProductTransition(prod_state, next_prod_state, edge.cond, skill_action));
                        // std::cout << "Added!!!" << endl;
                        // std::cout << "======================" << endl;
                        break;
                    }
                }
                // For garbage collection purposes.
                bdd_expr = bddfalse;
            }
        }

        // Add transitions based on "primitive" actions in the domain.
        for (const auto& action : grid_domain_.get_actions()) {
            GridState next_grid_state = grid_state.apply(action);
            if (!grid_domain_.is_valid_state(next_grid_state)) {
                continue;
            }
            // Get the atomic propositions at the next grid world state.
            set<string> next_grid_state_props = atomic_props(next_grid_state);
            if (curr_grid_state_props == next_grid_state_props) {
                // DFA state stays the same.
                // Any grid transition would be valid.
                ProductState next_prod_state(next_grid_state, dfa_state);
                product_transitions_[prod_state].push_back(ProductTransition(prod_state, next_prod_state, bddfalse, action));
            }
            next_grid_state_props.insert(curr_grid_state_props.begin(), curr_grid_state_props.end());
            // BDD operations
            bdd bdd_expr = props_to_bdd(next_grid_state_props);
            for (auto& edge: dfa_->out(dfa_state)) {

                // Could also use edge.acc, which is spot::acc_cond::mark_t
                if ((edge.cond & bdd_expr) == bdd_expr) { 
                    size_t next_dfa_state = edge.dst;
                    ProductState next_prod_state(next_grid_state, next_dfa_state);
                    product_transitions_[prod_state].push_back(ProductTransition(prod_state, next_prod_state, edge.cond, action));
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

    size_t dfa_start_state = dfa_->get_init_state_number();
    if (dfa_->state_is_accepting(dfa_start_state)) {
        cerr << "ERROR: DFA has a single state. LTL formula is not descriptive enough." << endl;
        return vector<ProductState>();
    }

    deque<ProductState> queue;
    queue.emplace_back(start_grid_state_, dfa_start_state);

    set<ProductState> visited;
    visited.insert(queue.front());

    map<ProductState, vector<ProductState>> parent_map;

    while (!queue.empty()) {
        ProductState current_state = queue.front();
        queue.pop_front();

        for (const auto& transition : product_transitions_[current_state]) {
            // check if the transition from `current_state` to `next_state` is accepting.
            ProductState next_state = transition.out_state();
            if (transition.path().size() > 1) {
                cout << "Adding Skill Action in the map!" << endl;
            }
            parent_map.emplace(next_state, transition.path());

            // Get the dfa states.
            size_t current_dfa_state = current_state.get_dfa_state();
            size_t next_dfa_state = next_state.get_dfa_state();

            // Check if this is not a "skill-action" transition and 
            // check if the dfa state changed. If so, add the new Skill-Action!
            if (transition.path().size() == 1 && current_dfa_state != next_dfa_state) {
                vector<ProductState> cached_path;
                cached_path.push_back(next_state);
                ProductState next_state_copy = next_state;
                // Backtrack to get the full transition path in the grid domain.
                while (parent_map.find(next_state_copy) != parent_map.end()) {
                    auto preceding_path = parent_map.at(next_state_copy);
                    if (preceding_path.front().get_dfa_state() != current_dfa_state) {
                        // We want to cache only a path from current_dfa_state to next_dfa_state for now.
                        break;
                    }
                    if (preceding_path.size() != 1) {
                        std::cout << "SkillAction is used in the path!" << endl;
                        std::cout << "Skill action contains the following path: " << endl;
                        if (!preceding_path.empty()) {
                            std::cout << preceding_path.front();
                            for (auto it = next(preceding_path.begin()); it != preceding_path.end(); ++it) {
                                std::cout << " -> " << *it;
                            }
                        }
                        std::cout << endl;
                        reverse(preceding_path.begin(), preceding_path.end());
                    }
                    cached_path.insert(cached_path.end(), preceding_path.begin(), preceding_path.end());
                    next_state_copy = preceding_path.back();
                }
                reverse(cached_path.begin(), cached_path.end());
                // TODO: apply some abstraction to cached path here!!! 
                if (use_skills_) {
                    bdd self_edge_cond = get_self_edge_cond(current_dfa_state);
                    SkillAction new_skill_action (cached_path, self_edge_cond, transition.dfa_edge_condition());
                    set<string> props = atomic_props(current_state.get_grid_state());
                    grid_domain_.add_skill_action(cached_path.front().get_grid_state(), new_skill_action);
                }
            }
            if (dfa_->state_is_accepting(next_dfa_state)) {
                product_path_.push_back(next_state);
                // Backtrack to get the full path.
                while (parent_map.find(next_state) != parent_map.end()) {
                    auto preceding_path = parent_map.at(next_state);
                    if (preceding_path.size() != 1) {
                        std::cout << "SkillAction is used in the path!" << endl;
                        std::cout << "Skill action contains the following path: " << endl;
                        if (!preceding_path.empty()) {
                            std::cout << preceding_path.front();
                            for (auto it = next(preceding_path.begin()); it != preceding_path.end(); ++it) {
                                std::cout << " -> " << *it;
                            }
                        }
                        std::cout << endl;
                        reverse(preceding_path.begin(), preceding_path.end());
                    }
                    product_path_.insert(product_path_.end(), preceding_path.begin(), preceding_path.end());
                    next_state = preceding_path.back();
                }
                reverse(product_path_.begin(), product_path_.end());
                save_paths();
                return product_path_;
            } else if (visited.find(next_state) == visited.end()) {
                queue.push_back(next_state);
                visited.insert(next_state);
            }
        }
    }
    return vector<ProductState>();
}

bdd TEGTask::get_self_edge_cond(size_t dfa_state) {
     for (auto& edge: dfa_->out(dfa_state)) {
        // Check if this is a self-edge.
        if (edge.dst == dfa_state) { 
            std::cout << "Self-transition was found!" << endl;
            return edge.cond;
        }
    }
    cerr << "ERROR: no condition for self-transition was found!" << endl;
    return bddfalse;
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

void TEGTask::print_dfa() {
    // We need the dictionary to print the BDDs that label the edges
    const spot::bdd_dict_ptr& dict = dfa_->get_dict();

    // Some meta-data...
    std::cout << "Acceptance: " << dfa_->get_acceptance() << '\n';
    std::cout << "Number of sets: " << dfa_->num_sets() << '\n';
    std::cout << "Number of states: " << dfa_->num_states() << '\n';
    std::cout << "Number of edges: " << dfa_->num_edges() << '\n';
    std::cout << "Initial state: " << dfa_->get_init_state_number() << '\n';
    std::cout << "Atomic propositions:";
    for (spot::formula ap: dfa_->ap()) {
        std::cout << ' ' << ap << " (=" << dict->varnum(ap) << ')';
    }
    std::cout << '\n';

    // Arbitrary data can be attached to automata, by giving them
    // a type and a name.  The HOA parser and printer both use the
    // "automaton-name" to name the automaton.
    if (auto name = dfa_->get_named_prop<string>("automaton-name")) {
        std::cout << "Name: " << *name << '\n';
    }

    // For the following prop_*() methods, the return value is an
    // instance of the spot::trival class that can represent
    // yes/maybe/no.  These properties correspond to bits stored in the
    // automaton, so they can be queried in constant time.  They are
    // only set whenever they can be determined at a cheap cost: for
    // instance an algorithm that always produces deterministic automata
    // would set the deterministic property on its output.  
    std::cout << "Complete: " << dfa_->prop_complete() << '\n';
    std::cout << "Deterministic: " << (dfa_->prop_universal()
                                && dfa_->is_existential()) << '\n';
    std::cout << "Unambiguous: " << dfa_->prop_unambiguous() << '\n';
    std::cout << "State-Based Acc: " << dfa_->prop_state_acc() << '\n';
    std::cout << "Terminal: " << dfa_->prop_terminal() << '\n';
    std::cout << "Weak: " << dfa_->prop_weak() << '\n';
    std::cout << "Inherently Weak: " << dfa_->prop_inherently_weak() << '\n';
    std::cout << "Stutter Invariant: " << dfa_->prop_stutter_invariant() << '\n';

    // States are numbered from 0 to n-1
    unsigned n = dfa_->num_states();
    for (unsigned s = 0; s < n; ++s) {
        std::cout << "State " << s << ":\n";

        // The out(s) method returns a fake container that can be
        // iterated over as if the contents was the edges going
        // out of s.  Each of these edges is a quadruplet
        // (src,dst,cond,acc).  Note that because this returns
        // a reference, the edge can also be modified.
        for (auto& t: dfa_->out(s)) {
            std::cout << "  edge(" << t.src << " -> " << t.dst << ")\n    label = ";
            spot::bdd_print_formula(std::cout, dict, t.cond);
            std::cout << "\n    acc sets = " << t.acc << '\n';
        }
    }
}

void TEGTask::print_product_transitions(int in_dfa_state, int out_dfa_state) {
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

void TEGTask::print_product_path() const {
    std::cout << "Product Path:" << endl;
    if (!product_path_.empty()) {
        std::cout << product_path_.front();
        for (auto it = next(product_path_.begin()); it != product_path_.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

void TEGTask::print_grid_path() const {
    std::cout << "Grid Path:" << endl;
    if (!grid_path_.empty()) {
        std::cout << grid_path_.front();
        for (auto it = next(grid_path_.begin()); it != grid_path_.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

void TEGTask::print_dfa_path() const {
    std::cout << "DFA Path:" << endl;
    if (!dfa_path_.empty()) {
        std::cout << dfa_path_.front();
        for (auto it = next(dfa_path_.begin()); it != dfa_path_.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

map<string, set<GridState>> TEGTask::get_ap_mapping() const {
    return formula_.get_ap_mapping();
}

string TEGTask::get_filename() const {
    return filename_;
}

