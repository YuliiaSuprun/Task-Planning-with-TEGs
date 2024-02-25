#include "TEGProblem.h"

#include <stdexcept>
#include <fstream>
#include <sstream>
#include <iostream>
#include <chrono>
#include <graphviz/gvc.h>

TEGProblem::TEGProblem(const string formula_str,
            const map<string, set<GridState>> ap_mapping,
            const GridWorldDomain& grid_domain,
            const GridState& start_grid_state,
            int problem_id, bool on_the_fly, bool cache, bool feedback)
    : grid_domain_(grid_domain), start_grid_state_(start_grid_state), problem_id_(problem_id), on_the_fly_(on_the_fly), cache_(cache),
    feedback_(feedback), bdd_dict_(make_shared<spot::bdd_dict>()),
    dfa_manager_(bdd_dict_) { 

    // Check if the start state is valid.
    if (!grid_domain_.is_valid_state(start_grid_state_)) {
        cerr << "ERROR: Invalid start state in the GridWorldDomain" << endl;
    }

    // Register all the propositions you need to bdd_dict.
    for (const auto& prop_pair : ap_mapping) {
        string prop = prop_pair.first;
        spot::formula prop_formula = spot::formula::ap(prop);
        bdd_dict_->register_proposition(prop_formula, nullptr);
    }
    // Print the "proposition to bdd" mapping.
    // bdd_dict_->dump(std::std::cout) << "---\n";

    std::cout << "Creating dfa for problem_id=" << problem_id_ << endl;
    // Get a name for output files.
    stringstream ss;
    ss << "dfa" << problem_id_;
    filename_ = ss.str();

    auto start1 = chrono::high_resolution_clock::now();

    // Convert to formula_str to LTLFormula object.
    formula_ = LTLFormula(formula_str, ap_mapping);

    // Compute the DFA corresponding to the given LTL formula.
    dfa_manager_.construct_dfa(formula_);

    auto end1 = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_dfa = end1 - start1;
    std::cout << "time of calculating the automaton: " << elapsed_dfa.count() << " seconds" << std::endl;

    // print_dfa();
    dfa_manager_.save_dfa(filename_);
    
    if (!on_the_fly_) {    
        auto start2 = chrono::high_resolution_clock::now();
        // Compute the product graph of DFA and GridWorldDomain.
        compute_product();

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

void TEGProblem::compute_product() {
    // Clear previous data.
    full_product_states_.clear();
    full_product_transitions_.clear();

    // Iterate over all grid world states and DFA states to compute product states.
    for (size_t r = 0; r < grid_domain_.R(); ++r) {
        for (size_t c = 0; c < grid_domain_.C(); ++c) {
            GridState grid_state(r, c);
            // States in dfa are always numbered from 0 to (num_states-1)
            for (size_t dfa_state = 0; dfa_state < dfa_manager_.get_num_states(); ++dfa_state) {
                ProductState ps(grid_state, dfa_state);
                full_product_states_.push_back(ps);
            }
        }
    }
    std::cout << full_product_states_.size() << " product states were generated" << endl;

    // Compute transitions in the product based on valid transitions in both the GridWorld and the DFA.
    for (const auto& prod_state : full_product_states_) {
        generate_successors(prod_state);
    }
}

set<string> TEGProblem::atomic_props(const GridState& grid_state) {
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
bdd TEGProblem::get_state_bdd(const GridState& grid_state) {

    // Start with a BDD representing true
    bdd result = bddtrue;

    // Iterate over the ap_mapping to determine whether each ap is + or -
    for (const auto& prop_pair : formula_.get_ap_mapping()) {
        string prop = prop_pair.first;
        spot::formula prop_formula = spot::formula::ap(prop);
        int var_num = bdd_dict_->varnum(prop_formula);
        if (var_num == -1) {
            cerr << "ERROR: proposition is not registered!" << endl;
        }
        bdd prop_bdd = bdd_ithvar(var_num);

        auto ap_states = prop_pair.second;

        if (ap_states.find(grid_state) == ap_states.end()) {
            // prop is absent (negative form)
            prop_bdd = bdd_not(prop_bdd);
        }   // Otherwise, prop has a positive form. Do nothing.

        // Logical AND with the result
        result &= prop_bdd;
    }

    return result;
}

vector<ProductState> TEGProblem::solve() {
    product_path_.clear();

    size_t dfa_start_state = dfa_manager_.get_start_state();
    if (dfa_manager_.is_accepting(dfa_start_state)) {
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

    size_t dfa_start_state = dfa_manager_.get_start_state();

    deque<ProductState> queue;
    queue.emplace_back(start_grid_state_, dfa_start_state);

    set<ProductState> visited;
    visited.insert(queue.front());

    map<ProductState, vector<ProductState>> parent_map;

    while (!queue.empty()) {
        ProductState current_state = queue.front();
        queue.pop_front();

        for (const auto& transition : full_product_transitions_[current_state]) {
            // Get the product state where this transition leads. 
            ProductState next_state = transition.out_state();

            // Get the next dfa state.
            size_t next_dfa_state = next_state.get_dfa_state();

            if (dfa_manager_.is_accepting(next_dfa_state)) {
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

shared_ptr<DFANode> TEGProblem::generate_dfa_path() {

    while (!nodePriorityQueue_.empty()) {
        auto currentNodePair = nodePriorityQueue_.top();
        nodePriorityQueue_.pop();
        auto currentNode = currentNodePair.second;
        // Delete the handle for this node in the map.
        node_handles_.erase(currentNode);

        size_t current_dfa_state = currentNode->getState();

        if (dfa_manager_.is_accepting(current_dfa_state)) {
            auto dfa_trace = currentNode->getPathFromRoot();
            cout << "Selected a DFA trace of size " << dfa_trace.size() - 1 << " and a cost of " << currentNodePair.first << endl;
            print_dfa_path(dfa_trace);
            return currentNode;
        }

        for (auto& edge: dfa_manager_.get_transitions(current_dfa_state)) {
            size_t next_dfa_state = edge.dst;

            if (next_dfa_state != current_dfa_state) { 
                // Calculate the cost for this transition
                int edge_cost = dfa_transition_cost(current_dfa_state, next_dfa_state);
                int total_cost = currentNodePair.first + edge_cost;

                auto childNode = make_shared<DFANode>(next_dfa_state, currentNode, edge_cost);
                currentNode->addChild(childNode);

                auto handle = nodePriorityQueue_.push(make_pair(total_cost, childNode));
                node_handles_[childNode] = handle;
            }
        }
    }
    return nullptr; // Return empty if no path is found
}


// Define the transition_cost function based on past experiences.
int TEGProblem::dfa_transition_cost(size_t from_state, size_t to_state) {
    if (!feedback_) return DEFAULT_COST;
    auto it = dfa_transition_costs_.find(make_pair(from_state, to_state));
    if (it != dfa_transition_costs_.end()) {
        // Return the stored cost if available
        return it->second;
    } else {
        // Default cost if the transition has not been attempted yet
        return DEFAULT_COST; // = 1
    }
}

void TEGProblem::update_dfa_transition_cost(shared_ptr<DFANode>& node, int cost) {

    if (!feedback_) {
        // We don't want to provide feedback when this flag is set to false.
        return;
    }

    auto parent = node->getParent();
    if (!parent) {
        // This is a root node.
        return;
    }

    // Update the cost in the cost table.
    dfa_transition_costs_[make_pair(parent->getState(), node->getState())] = cost;

    // Update the cost in the DFANode and the priority queue.
    node->updateParentEdgeCost(cost, node_handles_, nodePriorityQueue_);

    // print_node_priority_queue();
}


void TEGProblem::generate_successors(const ProductState& prod_state) {
    size_t dfa_state = prod_state.get_dfa_state();
    GridState grid_state = prod_state.get_grid_state();

    // Add transitions based on "primitive" actions in the domain.
    for (const auto& action : grid_domain_.get_actions()) {
        GridState next_grid_state = grid_state.apply(action);
        if (!grid_domain_.is_valid_state(next_grid_state)) {
            continue;
        }
        auto dfa_transition = find_transition(next_grid_state, dfa_state);

        if (dfa_transition == nullptr) {
            continue;
        }

        size_t next_dfa_state = dfa_transition->dst;
        ProductState next_prod_state(next_grid_state, next_dfa_state);
        auto grid_action = make_shared<GridAction>(action);
        full_product_transitions_[prod_state].push_back(ProductTransition(prod_state, next_prod_state, dfa_transition->cond, grid_action));
    }
}

bool TEGProblem::is_transition_valid(const bdd& edge_cond, const bdd& next_state_bdd) {
    // next_state_bdd is a full assignment
    // edge_cond is a partial assignment
    // check if "next_state_bdd => edge_cond"
    bdd implication = bdd_apply(bdd_not(next_state_bdd), edge_cond, bddop_or);
    return implication == bddtrue;
}

spot::twa_graph::edge_storage_t* TEGProblem::find_transition(const GridState& next_grid_state, size_t curr_dfa_state) {
    // Retrieve atomic propositions for next grid states
    bdd next_grid_state_bdd = get_state_bdd(next_grid_state);

    // Iterate over outgoing transitions of the current DFA state
    for (auto& edge : dfa_manager_.get_transitions(curr_dfa_state)) {
        if (is_transition_valid(edge.cond, next_grid_state_bdd)) {
            // Found a valid transition
            return &edge;
        }
    }
    // If no valid transition is found
    return nullptr;
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
    ProductState start_state(start_grid_state_, dfa_start_state);
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
        if (full_product_transitions_.count(current_state) == 0) {
            // Expand the state.
            generate_successors(current_state);
        }

        for (const auto& transition : full_product_transitions_.at(current_state)) {
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
                update_dfa_transition_cost(dfa_nodes.at(currentRegionIndex + 1), SUCCESS_COST);

                // Optimization 1: cache paths that cross the dfa states
                if (cache_ && !transition.isCached()) {

                    vector<ProductState> path_to_cache_reversed = construct_path(parent_map, next_state, true, curr_dfa_state);
                
                    // Find the source node of this transition.
                    ProductState start_state = path_to_cache_reversed.back();

                    // Create compound action.
                    auto compound_action = make_shared<CompoundAction>(path_to_cache_reversed);
                    // Now we create a transition and add it to a product graph.
                    // Serves as a "skip-connection".
                    full_product_transitions_[start_state].push_back(ProductTransition(start_state, next_state, transition.dfa_edge_condition(), compound_action, true));
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
    update_dfa_transition_cost(dfa_nodes.at(maxRegionIndexReached + 1), FAILURE_COST);
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

void TEGProblem::solve_with_on_the_fly_graph() {

    full_product_transitions_.clear();
    dfa_path_.clear();

    // Initialize all paths from the root = the start dfa state.
    auto root = make_shared<DFANode>(dfa_manager_.get_start_state());
    auto handle = nodePriorityQueue_.push(make_pair(0, root));
    node_handles_[root] = handle;


    int max_num_iters = 50; // Depends on the nature of the problem.
    int curr_iter = 0;

    while (product_path_.empty() && curr_iter < max_num_iters) {
        cout << "=== Iteration " << curr_iter++ << " ===" << endl;
        // Pick a "likely" DFA trace that ends in the acceptance state.
        auto endTraceNode = generate_dfa_path();

        if (!endTraceNode) {
            // No more accepted DFA traces were found.
            break;
        }

        // Attempt to realize this trace in the task domain.
        realize_dfa_trace(endTraceNode);
    }
}

void TEGProblem::save_paths() {
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

vector<GridState> TEGProblem::get_grid_path() const {
    return grid_path_;
}

vector<size_t> TEGProblem::get_dfa_path() const {
    return dfa_path_;
}

void TEGProblem::print_product_transitions(int in_dfa_state, int out_dfa_state) {
    std::cout << "Product Transitions:" << endl;
    for (const auto& transition_entry : full_product_transitions_) {
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

void TEGProblem::print_grid_path() const {
    std::cout << "Grid Path:" << endl;
    if (!grid_path_.empty()) {
        std::cout << grid_path_.front();
        for (auto it = next(grid_path_.begin()); it != grid_path_.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

void TEGProblem::print_dfa_path() const {
    std::cout << "DFA Path:" << endl;
    if (!dfa_path_.empty()) {
        std::cout << dfa_path_.front();
        for (auto it = next(dfa_path_.begin()); it != dfa_path_.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

void TEGProblem::print_dfa_path(vector<size_t> dfa_path) const {
    std::cout << "DFA Path:" << endl;
    if (!dfa_path.empty()) {
        std::cout << dfa_path.front();
        for (auto it = next(dfa_path.begin()); it != dfa_path.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

map<string, set<GridState>> TEGProblem::get_ap_mapping() const {
    return formula_.get_ap_mapping();
}

string TEGProblem::get_filename() const {
    return filename_;
}

void TEGProblem::print_node_priority_queue() { 
    cout << "Printing the priority queue..." << endl;
    NodeHeap heapCopy = nodePriorityQueue_;  // Make a copy of the heap
    while (!heapCopy.empty()) {
        auto pair = heapCopy.top();
        int cost = pair.first;
        auto node = pair.second;
        // Print 'cost' and any information from 'node'
        cout << "Cost: " << cost << ", DFA state: " << node->getState() << ", cost in the DFANode: " << node->getPathCost() << endl;
        heapCopy.pop();
    }
}

