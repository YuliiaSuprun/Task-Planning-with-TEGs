#include "PDDLProblem.h"
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>
#include <iostream>

PDDLProblem::PDDLProblem(const string& problemFile, shared_ptr<PDDLDomain> domainPtr, bool cache, bool feedback, bool use_landmarks, int problem_id) 
: domain_(domainPtr), problem_id_(problem_id), cache_(cache),
    use_landmarks_(use_landmarks),
    bdd_dict_(make_shared<spot::bdd_dict>()) {

    parseProblem(problemFile, domainPtr);
    cout << "Problem was parsed!!!" << endl;
    pddlProblem_->toPDDL(std::cout) << std::endl;

    // Set a start position.
    start_domain_state_ = make_shared<PDDLState>(pddlProblem_->start);

    map<string, pair<string, vector<string>>> pred_mapping;
    // Set a mapping from propositions to grounded predicates.
    pddlProblem_->goal->getAtomicPropsMap(pred_mapping);

    // Initialize a dfa manager.
    dfa_manager_ = make_shared<DFAManager>(bdd_dict_, feedback);

    std::cout << "Creating dfa for problem_id=" << problem_id_ << endl;
    // Get a name for output files.
    stringstream ss;
    ss << "pddlDfa" << problem_id_;
    filename_ = ss.str();

    auto start1 = chrono::high_resolution_clock::now();

    // Convert to formula_str to LTLFormula object.
    formula_ = LTLFormula(pddlProblem_->goal->toLTL(), pred_mapping);

    // Compute the DFA corresponding to the given LTL formula.
    dfa_manager_->construct_dfa(formula_);

    auto end1 = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed_dfa = end1 - start1;
    std::cout << "time of calculating the automaton: " << elapsed_dfa.count() << " seconds" << std::endl;

    // dfa_manager_->print_dfa();
    // dfa_manager_->save_dfa(filename_);

    // Initialize domain and product managers.
    domain_manager_ = make_shared<DomainManager>(bdd_dict_, domain_, get_pred_mapping());

    product_manager_ = make_shared<ProductManager>(domain_manager_, dfa_manager_);
}

PDDLProblem::~PDDLProblem() {
    // Unregister all propositions: need for memory management.
    bdd_dict_->unregister_all_my_variables(nullptr);
    // When the reference count drops to zero, the destructor for the spot::bdd_dict will be triggered.
}

void PDDLProblem::parseProblem(const std::string& problemFile, std::shared_ptr<PDDLDomain> domainPtr) {
    auto pddlboatDomainPtr = domainPtr->getPddlboatDomainPtr();

    pddlboat::ast::Problem problem_ast;
    if (!pddlboat::parseFile(problemFile, problem_ast)) {
        throw std::runtime_error("Failed to parse PDDL problem file: " + problemFile);
    }

    try {
        pddlProblem_ = pddlboat::toProblem(problem_ast, pddlboatDomainPtr);
    } catch (const std::exception& e) {
        throw std::runtime_error("Exception translating problem: " + std::string(e.what()));
    }
}

const pddlboat::ProblemPtr& PDDLProblem::getPddlboatProblemPtr() const {
    return pddlProblem_;
}

vector<ProductState> PDDLProblem::solve() {
    product_path_.clear();
    dfa_path_.clear();

    size_t dfa_start_state = dfa_manager_->get_start_state();
    if (dfa_manager_->is_accepting_state(dfa_start_state)) {
        cerr << "ERROR: DFA has a single state. LTL formula is not descriptive enough." << endl;
        return vector<ProductState>();
    }

    // Initialize all paths from the root = the start dfa state.
    dfa_manager_->initialize_node_priority_queue();

    int max_num_iters = 50; // Depends on the nature of the problem.
    int curr_iter = 0;

    while (product_path_.empty() && curr_iter < max_num_iters) {
        cout << "=== Iteration " << curr_iter++ << " ===" << endl;
        // domain_manager_->print_ap_mapping();
        // Pick a "likely" DFA trace that ends in the acceptance state.
        auto endTraceNode = dfa_manager_->generate_dfa_path();

        if (!endTraceNode) {
            // No more accepted DFA traces were found.
            cout << "endTraceNode is null" << endl;
            break;
        }

        // Attempt to realize this trace in the task domain.
        realize_dfa_trace(endTraceNode);
    }
    return product_path_;
}

void PDDLProblem::realize_dfa_trace(shared_ptr<DFANode>& endTraceNode) {

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

    size_t currentRegionIndex = 0;
    size_t maxRegionIndexReached = 0;

    set<ProductState> visited;
    visited.insert(start_state);

    // Initialize a map of priority queues 
    // (one queue for each DFA state in the trace)
    map<size_t, priority_queue<pair<int, ProductState>, vector<pair<int, ProductState>>, greater<pair<int, ProductState>>>> regionQueues;

    int curr_state_heuristic = 0;

    // We need this for computing heuristics.
    map<size_t, DomainStateSet> landmarks;

    if (use_landmarks_) {
        auto next_dfa_edge_condition = dfa_nodes.at(1)->getParentEdgeCondition();
        landmarks[dfa_start_state] = product_manager_->sample_landmarks(next_dfa_edge_condition, start_domain_state_);

        curr_state_heuristic = start_state.compute_heuristic_cost(landmarks[dfa_start_state]);
    }

    regionQueues[dfa_start_state].push(make_pair(curr_state_heuristic, start_state));

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
        auto current_state_pair = queue.top();
        ProductState current_state = current_state_pair.second;
        // cout << "Popped the state " << current_state << " with score " << current_state_pair.first << endl;
        queue.pop();

        // Check if state was expanded?
        // If not, then generate successors for this state!
        if (product_manager_->state_not_expanded(current_state)) {
            // Expand the state.
            product_manager_->generate_successors(current_state);
        }

        for (auto& transition : product_manager_->get_transitions(current_state)) {
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
                // cout << "SKIP: an illegal DFA transition: " << curr_dfa_state << "=>" << next_dfa_state << endl;
                // Skip states that lead to any other dfa states.
                continue;
            }

            // cout << "Transitioning: " << curr_dfa_state << "=>" << next_dfa_state << endl;

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

                    // TODO: cache it as a path between modalities in DomainManager (can be reused for other TEG problems).
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

                // Compute landmarks for the next DFA region:
                // 1) Sample k points in the next equivalence region (aka landmarks). 
                // 2) Define heuristic as a distance to the closest sampled point.
                // 3) Not an admissible heuristic but we don't care about optimality of the path, so it's fine.

                if (use_landmarks_ && landmarks.count(next_dfa_state) == 0) {
                    auto next_dfa_edge_condition = dfa_nodes.at(currentRegionIndex + 1)->getParentEdgeCondition();
                    landmarks[next_dfa_state] = product_manager_->sample_landmarks(next_dfa_edge_condition, next_state.get_domain_state());
                    if (landmarks[next_dfa_state].empty()) {
                        dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(maxRegionIndexReached + 1), FAILURE_COST);
                        return;
                    }
                }

            } 
            // Compute the heuristic score for this state and push it into the queue.
            int curr_state_cost = current_state_pair.first;
            // Default cost of an edge is 1.
            int next_state_cost = curr_state_cost + DEFAULT_COST; 
            if (use_landmarks_) {
                int curr_heuristic_cost = current_state.compute_heuristic_cost(landmarks[curr_dfa_state]);
                int next_heuristic_cost = next_state.compute_heuristic_cost(landmarks[next_dfa_state]);
                next_state_cost += (next_heuristic_cost - curr_heuristic_cost);
            }

            // Push this new product state onto the corresponding queue.
            regionQueues[next_dfa_state].push(make_pair(next_state_cost, next_state));
            // This state has been visited now!
            visited.insert(next_state);
        }
    }
    // We failed to realize the provided dfa trace.
    // Update the cost for the dfa transition we couldn't realize.
    dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(maxRegionIndexReached + 1), FAILURE_COST);
}

vector<ProductState> PDDLProblem::construct_path(const map<ProductState, vector<ProductState>>& parent_map, ProductState target_state, bool cached, size_t start_dfa_state) {
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

void PDDLProblem::save_paths() {
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

vector<shared_ptr<DomainState>> PDDLProblem::get_domain_path() const {
    return domain_path_;
}

vector<size_t> PDDLProblem::get_dfa_path() const {
    return dfa_path_;
}

void PDDLProblem::print_product_path() const {
    std::cout << "Product Path:" << endl;
    if (!product_path_.empty()) {
        std::cout << product_path_.front();
        for (auto it = next(product_path_.begin()); it != product_path_.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

void PDDLProblem::print_domain_path() const {
    std::cout << "Domain Path:" << endl;
    if (!domain_path_.empty()) {
        std::cout << *domain_path_.front();
        for (auto it = next(domain_path_.begin()); it != domain_path_.end(); ++it) {
            std::cout << " -> " << **it;
        }
    }
    std::cout << endl;
}

void PDDLProblem::print_dfa_path() const {
    dfa_manager_->print_dfa_path(dfa_path_);
}

string PDDLProblem::get_filename() const {
    return filename_;
}

map<string, pair<string, vector<string>>> PDDLProblem::get_pred_mapping() const {
    return formula_.get_pred_mapping();
}