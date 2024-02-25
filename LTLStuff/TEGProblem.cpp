#include "TEGProblem.h"

#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>

TEGProblem::TEGProblem(const string formula_str,
            const map<string, set<GridState>> ap_mapping,
            const shared_ptr<GridWorldDomain> domain,
            const GridState& start_domain_state,
            int problem_id, bool on_the_fly, bool cache, bool feedback)
    : domain_(domain), start_domain_state_(start_domain_state), problem_id_(problem_id), on_the_fly_(on_the_fly), cache_(cache),
    bdd_dict_(make_shared<spot::bdd_dict>()),
    dfa_manager_(make_shared<DFAManager>(bdd_dict_, feedback)),
    domain_manager_(make_shared<DomainManager>(bdd_dict_, domain_, ap_mapping)),
    product_manager_(make_shared<ProductManager>(domain_manager_, dfa_manager_))
    { 

    // Check if the start state is valid.
    if (!domain_->is_valid_state(start_domain_state_)) {
        cerr << "ERROR: Invalid start state in the task domain!" << endl;
    }

    std::cout << "Creating dfa for problem_id=" << problem_id_ << endl;
    // Get a name for output files.
    stringstream ss;
    ss << "dfa" << problem_id_;
    filename_ = ss.str();

    auto start1 = chrono::high_resolution_clock::now();

    // Convert to formula_str to LTLFormula object.
    formula_ = LTLFormula(formula_str, ap_mapping);

    // Compute the DFA corresponding to the given LTL formula.
    dfa_manager_->construct_dfa(formula_);

    auto end1 = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_dfa = end1 - start1;
    std::cout << "time of calculating the automaton: " << elapsed_dfa.count() << " seconds" << std::endl;

    // dfa_manager_->print_dfa();
    dfa_manager_->save_dfa(filename_);
    
    if (!on_the_fly_) {    
        auto start2 = chrono::high_resolution_clock::now();
        // Compute the product graph of DFA and GridWorldDomain.
        product_manager_->compute_full_product();

        auto end2 = chrono::high_resolution_clock::now();
        chrono::duration<double> elapsed_product = end2 - start2;
        std::cout << "time of calculating the product: " << elapsed_product.count() << " seconds" << std::endl;
    }
}

TEGProblem::~TEGProblem() {
    // Unregister all propositions: need for memory management.
    bdd_dict_->unregister_all_my_variables(nullptr);
    // When the reference count drops to zero, the destructor for the spot::bdd_dict will be triggered.
}

vector<ProductState> TEGProblem::solve() {
    product_path_.clear();

    size_t dfa_start_state = dfa_manager_->get_start_state();
    if (dfa_manager_->is_accepting_state(dfa_start_state)) {
        cerr << "ERROR: DFA has a single state. LTL formula is not descriptive enough." << endl;
        return vector<ProductState>();
    }

    if (on_the_fly_) {
        solve_with_on_the_fly_graph();
    } else {
        solve_with_full_graph();
    }
    return product_path_;
}

void TEGProblem::solve_with_full_graph() {

    size_t dfa_start_state = dfa_manager_->get_start_state();

    deque<ProductState> queue;
    queue.emplace_back(start_domain_state_, dfa_start_state);

    set<ProductState> visited;
    visited.insert(queue.front());

    map<ProductState, vector<ProductState>> parent_map;

    while (!queue.empty()) {
        ProductState current_state = queue.front();
        queue.pop_front();

        for (const auto& transition : product_manager_->get_transitions(current_state)) {
            // Get the product state where this transition leads. 
            ProductState next_state = transition.out_state();

            // Get the next dfa state.
            size_t next_dfa_state = next_state.get_dfa_state();

            if (dfa_manager_->is_accepting_state(next_dfa_state)) {
                // Case 1: This next state is accepting. Solution is found!
                parent_map.emplace(next_state, transition.path());
                product_path_ = construct_path(parent_map, next_state);
                save_paths();
                return;
            } else if (visited.find(next_state) == visited.end()) {
                // Case 2: This product state was not visited yet.
                // Push it onto the queue for further exploration. 
                queue.push_back(next_state);
                visited.insert(next_state);
                parent_map.emplace(next_state, transition.path());
            }
                // Case 3: This product state was already visited. Do nothing.
        }
    }
}

void TEGProblem::solve_with_on_the_fly_graph() {

    dfa_path_.clear();

    // Initialize all paths from the root = the start dfa state.
    dfa_manager_->initialize_node_priority_queue();

    int max_num_iters = 50; // Depends on the nature of the problem.
    int curr_iter = 0;

    while (product_path_.empty() && curr_iter < max_num_iters) {
        cout << "=== Iteration " << curr_iter++ << " ===" << endl;
        // Pick a "likely" DFA trace that ends in the acceptance state.
        auto endTraceNode = dfa_manager_->generate_dfa_path();

        if (!endTraceNode) {
            // No more accepted DFA traces were found.
            break;
        }

        // Attempt to realize this trace in the task domain.
        realize_dfa_trace(endTraceNode);
    }
}

void TEGProblem::realize_dfa_trace(shared_ptr<DFANode>& endTraceNode) {

    auto dfa_trace = endTraceNode->getPathFromRoot();

    auto dfa_nodes = endTraceNode->getNodePathFromRoot();

    if (dfa_trace.empty()) {
        cerr << "ERROR: the DFA trace is empty!" << endl;
        return;
    } else if (dfa_trace.size() == 1) {
        cerr << "ERROR: the DFA trace is of size 1!" << endl;
        return;
    }

    size_t dfa_start_state = dfa_trace.front();
    ProductState start_state(start_domain_state_, dfa_start_state);
    map<size_t, deque<ProductState>> regionQueues;
    regionQueues[dfa_start_state].push_back(start_state);

    size_t currentRegionIndex = 0;
    size_t maxRegionIndexReached = 0;

    set<ProductState> visited;
    visited.insert(start_state);

    map<ProductState, vector<ProductState>> parent_map;

    while (!regionQueues[dfa_start_state].empty()) {
        size_t curr_dfa_state = dfa_trace.at(currentRegionIndex);
        
        auto& queue = regionQueues[curr_dfa_state];

        if (queue.empty()) {
            // Backtrack if necessary
            currentRegionIndex--;
            continue;
        }
        if (queue.size() == 1) {
            cout << "Trying to realize this transition: " << curr_dfa_state << "=>" << dfa_trace.at(currentRegionIndex+1) << endl;
        }
        // Queue is not empty!
        ProductState current_state = queue.front();
        queue.pop_front();

        // Check if state was expanded?
        // If not, then generate successors for this state!
        if (product_manager_->state_not_expanded(current_state)) {
            // Expand the state.
            product_manager_->generate_successors(current_state);
        }

        for (const auto& transition : product_manager_->get_transitions(current_state)) {
            // Get the product state where this transition leads. 
            ProductState next_state = transition.out_state();

            // Check if it was already explored.
            if (visited.find(next_state) != visited.end()) {
                // Skip this state.
                continue;
            }

            // Get the next dfa state.
            size_t next_dfa_state = next_state.get_dfa_state();

            // We are interested only in the transitions that either stay in the same dfa state or lead to the next dfa state in the trace.
            if (next_dfa_state != dfa_trace.at(currentRegionIndex) &&
            next_dfa_state != dfa_trace.at(currentRegionIndex + 1)) {
                // Skip states that lead to any other dfa states.
                continue;
            }

            // Add it to the parent map to be able to backtrack.
            parent_map.emplace(next_state, transition.path());
 
            if (next_dfa_state == dfa_trace.at(currentRegionIndex + 1)) {
                // We were able to get to the next DFA state in a trace!
                cout << "Succesfully realized this transition: " << transition.in_state().get_dfa_state() << "=>" << transition.out_state().get_dfa_state() << endl; 

                // Update the cost to 0 based on the success of the transition.
                dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(currentRegionIndex + 1), SUCCESS_COST);

                // Optimization 1: cache paths that cross the dfa states
                if (cache_ && !transition.isCached()) {

                    vector<ProductState> path_to_cache_reversed = construct_path(parent_map, next_state, true, curr_dfa_state);

                    // Now we create a transition and add it to a product graph.
                    // Serves as a "skip-connection".
                    product_manager_->cache_path(path_to_cache_reversed, transition.dfa_edge_condition());
                }

                currentRegionIndex++;

                // Remember the last dfa state we were able to reach.
                if (maxRegionIndexReached < currentRegionIndex) {
                    maxRegionIndexReached = currentRegionIndex;
                }
                // Check if it's the accepting state (last in the dfa trace).
                // If so, we have a solution!
                if (currentRegionIndex == dfa_trace.size() - 1) {
                    // Backtrack to get the full path.
                    product_path_ = construct_path(parent_map, next_state);
                    save_paths();
                    return;
                }

            } 

            // Push this new product state onto the corresponding queue.
            regionQueues[next_dfa_state].push_back(next_state);
            // This state has been visited now!
            visited.insert(next_state);
        }
    }
    // We failed to realize the provided dfa trace.
    // Update the cost for the dfa transition we couldn't realize.
    dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(maxRegionIndexReached + 1), FAILURE_COST);
}

vector<ProductState> TEGProblem::construct_path(const map<ProductState, vector<ProductState>>& parent_map, ProductState target_state, bool cached, size_t start_dfa_state) {
    vector<ProductState> path;
    path.push_back(target_state);

    while (parent_map.find(target_state) != parent_map.end()) {

        const auto& preceding_path = parent_map.at(target_state);

        // if (preceding_path.size() != 1) {
        //     std::cout << "Cached path is used!" << endl;
        // }

        target_state = preceding_path.back();

        if (cached && target_state.get_dfa_state() != start_dfa_state) {
            // We only want to cache 1-step dfa transitions for now.
            break;
        } 

        path.insert(path.end(), preceding_path.begin(), preceding_path.end());
    }

    if (!cached) {
        reverse(path.begin(), path.end());
    }
    return path;
}

void TEGProblem::save_paths() {
    domain_path_.clear();
    dfa_path_.clear();
    int prev_dfa_state = -1;
    for(const auto& ps : product_path_) {
        domain_path_.push_back(ps.get_domain_state());
        int curr_dfa_state = ps.get_dfa_state();
        if (prev_dfa_state != curr_dfa_state) {
            dfa_path_.push_back(curr_dfa_state);
            prev_dfa_state = curr_dfa_state;
        }
    }
}

vector<GridState> TEGProblem::get_domain_path() const {
    return domain_path_;
}

vector<size_t> TEGProblem::get_dfa_path() const {
    return dfa_path_;
}

void TEGProblem::print_product_path() const {
    std::cout << "Product Path:" << endl;
    if (!product_path_.empty()) {
        std::cout << product_path_.front();
        for (auto it = next(product_path_.begin()); it != product_path_.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

void TEGProblem::print_domain_path() const {
    std::cout << "Grid Path:" << endl;
    if (!domain_path_.empty()) {
        std::cout << domain_path_.front();
        for (auto it = next(domain_path_.begin()); it != domain_path_.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

void TEGProblem::print_dfa_path() const {
    dfa_manager_->print_dfa_path(dfa_path_);
}

map<string, set<GridState>> TEGProblem::get_ap_mapping() const {
    return formula_.get_ap_mapping();
}

string TEGProblem::get_filename() const {
    return filename_;
}
