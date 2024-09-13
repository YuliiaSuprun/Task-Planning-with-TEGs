#include "PDDLProblem.h"
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>
#include <iostream>
#include <sstream>
#include <cassert>
#include <fstream>
#include <filesystem>  // Requires C++17

PDDLProblem::PDDLProblem(const string& problemFile, shared_ptr<PDDLDomain> domainPtr, const string& planner_type, bool cache, bool feedback, bool use_landmarks, bool hamming_dist, bool save_dfa) 
: domain_(domainPtr), planner_type_(planner_type), cache_(cache),
feedback_(feedback), use_landmarks_(use_landmarks),
hamming_dist_(hamming_dist), save_dfa_(save_dfa), bdd_dict_(make_shared<spot::bdd_dict>()), num_expanded_nodes_(0), num_of_backtracks_(0) {

    // Extracting the problem name
    extract_names(problemFile);

    parseProblem(problemFile, domainPtr);
    // cout << "Problem was parsed!!!" << endl;
    // pddlProblem_->toPDDL(std::cout) << std::endl;

    // Set a start position.
    start_domain_state_ = make_shared<PDDLState>(pddlProblem_->start);

    map<string, pair<string, vector<string>>> pred_mapping;
    // Set a mapping from propositions to grounded predicates.
    pddlProblem_->goal->getAtomicPropsMap(pred_mapping);
    // for (const auto& p: pred_mapping) {
    //     cout << "Pred is " << p.first << endl;
    // }
    // cout << "pred_mapping size is " << pred_mapping.size() << endl;


    // Initialize a dfa manager.
    dfa_manager_ = make_shared<DFAManager>(bdd_dict_, feedback_, hamming_dist_);

    // std::cout << "Creating a DFA for the problem " << problem_name_ << endl;
    // Create a name for output files.
    filename_ = problem_name_ + "_pddl_dfa";

    // std::cout << "The output file will be named: " << filename_ << endl;

    auto start1 = chrono::high_resolution_clock::now();

    // Convert to formula_str to LTLFormula object.
    formula_ = LTLFormula(pddlProblem_->goal->toLTL(), pred_mapping);

    // cout << "Formula: " << formula_ << endl;

    // Compute the DFA corresponding to the given LTL formula.
    dfa_manager_->construct_dfa(formula_);

    auto end1 = chrono::high_resolution_clock::now();
    dfa_construction_time_ = end1 - start1;
    // std::cout << "time of calculating the automaton: " << dfa_construction_time_.count() << " seconds" << std::endl;

    // dfa_manager_->print_dfa();
    if (save_dfa_) {
        dfa_manager_->save_dfa(filename_, domain_name_);
    }
    // cout << "DFA was generated" << endl;

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
        // cout << "=== Iteration " << curr_iter++ << " ===" << endl;
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

    if (planner_type_ != "manual") {
        // 1. Create an instanse of the subproblem 
        // 2. Call the off-the-shelf planner
        realize_dfa_trace_with_planner(endTraceNode);
    } else {
        realize_dfa_trace_manually(endTraceNode);
    }
}

void PDDLProblem::realize_dfa_trace_manually(shared_ptr<DFANode>& endTraceNode) {

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
    size_t max_num_wrong_dfa_trans = 300;

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

    size_t curr_num_wrong_dfa_trans = 0;

    while (!regionQueues[dfa_start_state].empty()) {
        size_t curr_dfa_state = dfa_trace.at(currentRegionIndex);
        
        auto& queue = regionQueues[curr_dfa_state];

        if (queue.empty()) {
            // Backtrack if necessary
            currentRegionIndex--;
            continue;
        }
        // if (queue.size() == 1) {
        //     cout << "Trying to realize this transition: " << curr_dfa_state << "=>" << dfa_trace.at(currentRegionIndex+1) << endl;
        // }
        // Queue is not empty!
        auto current_state_pair = queue.top();
        ProductState current_state = current_state_pair.second;
        // cout << "Popped the state " << current_state << " with score " << current_state_pair.first << endl;
        queue.pop();

        // Check if state was expanded?
        // If not, then generate successors for this state!
        if (product_manager_->state_not_expanded(current_state)) {
            num_expanded_nodes_++;
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
                // Update cost for this transition to 0.
                if (feedback_) {
                    dfa_manager_->update_dfa_transition_cost(curr_dfa_state, next_dfa_state, SUCCESS_COST);
                }

                curr_num_wrong_dfa_trans++;
                // cout << "curr_num_wrong_dfa_trans = " << curr_num_wrong_dfa_trans << endl;
                if (curr_num_wrong_dfa_trans > max_num_wrong_dfa_trans) {
                    // Give up on this trace!
                    dfa_manager_->give_up_on_path(dfa_nodes.back(), dfa_nodes.at(currentRegionIndex+1));
                    num_of_backtracks_++;
                    cout << "Giving up on transition: " << curr_dfa_state << "=>" << dfa_trace.at(currentRegionIndex+1) << endl;
                    return;
                }
                // if (feedback_ && dfa_manager_->dfa_transition_cost(curr_dfa_state, next_dfa_state) != SUCCESS_COST) {
                //     dfa_manager_->update_dfa_transition_cost(curr_dfa_state, next_dfa_state, SUCCESS_COST);
                //     // Cache this for future reuse.
                //     if (cache_ && !transition.isCached()) {
                //         // cout << "Caching it for future! " << endl;
                //         vector<ProductState> path_to_cache_reversed = construct_path(parent_map, next_state, true, curr_dfa_state);
                //         // Now we create a transition and add it to a product graph.
                //         // Serves as a "skip-connection".
                //         product_manager_->cache_path(path_to_cache_reversed, transition.dfa_edge_condition());
                //     }
                // }
                continue;
            }

            // cout << "Transitioning: " << curr_dfa_state << "=>" << next_dfa_state << endl;

            // Add it to the parent map to be able to backtrack.
            parent_map.emplace(next_state, transition.path());
 
            if (next_dfa_state == dfa_trace.at(currentRegionIndex + 1)) {
                // We were able to get to the next DFA state in a trace!
                // cout << "Succesfully realized this transition: " << transition.in_state().get_dfa_state() << "=>" << transition.out_state().get_dfa_state() << endl; 

                curr_num_wrong_dfa_trans = 0;

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

void PDDLProblem::realize_dfa_trace_with_planner(shared_ptr<DFANode>& endTraceNode) {

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
    size_t curr_dfa_state = dfa_trace.at(currentRegionIndex);
    size_t next_dfa_state = dfa_trace.at(currentRegionIndex + 1);

    int max_num_wrong_dfa_trans = 3;
    int max_num_repeated_paths = 3;

    set<ProductState> visited;
    visited.insert(start_state);

    // Initialize a map of subproblems?
    // (one subproblem for each DFA state in the trace)
    map<size_t, pddlboat::ProblemPtr> regionSubproblems;

    // int curr_state_heuristic = 0;
    // Instantiate a subproblem (pddlboat::ProblemPtr)
    pddlboat::ProblemPtr start_subproblem = create_subproblem(dfa_nodes.at(1)->getParentEdgeCondition(), start_domain_state_);
    regionSubproblems.insert({dfa_start_state, start_subproblem});

    map<ProductState, vector<ProductState>> parent_map;
    
    int num_wrong_dfa_trans = 0;
    int num_repeated_paths = 0;
    while (!regionSubproblems.empty()) {
        curr_dfa_state = dfa_trace.at(currentRegionIndex);
        next_dfa_state = dfa_trace.at(currentRegionIndex + 1);
        

        // cout << "Trying to realize a transition: " << curr_dfa_state << "=>" << next_dfa_state << endl;
        if (regionSubproblems.count(curr_dfa_state) == 0) {
            // This problem can't be solved.
            // cout << "Backtracking" << endl;
            // Backtrack if necessary
            assert(currentRegionIndex > 0 && "currentRegionIndex must be positive");
            currentRegionIndex--;
            continue;  
        }

        auto& subproblem = regionSubproblems[curr_dfa_state];
        std::cout << "Subproblem to be solved: " << std::endl;
        subproblem->goal->toPDDL(std::cout) << std::endl;

        auto curr_domain_state = make_shared<PDDLState>(subproblem->start);
        ProductState curr_prod_state(curr_domain_state, curr_dfa_state);

        // Initial values (needed for working with cached solutions)
        auto last_domain_state = curr_domain_state;
        auto next_prod_state = curr_prod_state;
        bool retrieved_path = false;

        if (cache_) {
            // Retrieve the cached solution if one exists.
            for (auto &transition : product_manager_->get_transitions(curr_prod_state)) {
                next_prod_state = transition.out_state();
                
                if (next_prod_state.get_dfa_state() == next_dfa_state) {
                    // Add it to the parent map to be able to backtrack.
                    cout << "WARNING: Using a cached solution!" << endl;
                    parent_map.emplace(next_prod_state, transition.path());
                    last_domain_state = static_pointer_cast<PDDLState>(next_prod_state.get_domain_state());
                    retrieved_path = true;
                }
            }
        }

        if (!retrieved_path) {
            // Instantiate the planner based on the type
            shared_ptr<pddlboat::Planner> task_planner;

            if (planner_type_ == "z3") {
                pddlboat::Z3Planner::Options options;
                options.dump_clauses = false;
                task_planner = make_shared<pddlboat::Z3Planner>(subproblem, options);
            } else if (planner_type_ == "fd") {
                pddlboat::FastDownwardPlanner::Options options;
                options.search = pddlboat::FastDownwardPlanner::LAZYGREEDY;  // LAZYGREEDY, FDAT, LAMA
                task_planner = make_shared<pddlboat::FastDownwardPlanner>(subproblem, options);
            } else if (planner_type_ == "ff") {
                task_planner = make_shared<pddlboat::FastForwardPlanner>(subproblem);
            } else if (planner_type_ == "symk") {
                task_planner = make_shared<pddlboat::SymKPlanner>(subproblem);
            } else if (planner_type_ == "astar") {
                task_planner = make_shared<pddlboat::AStarPlanner>(subproblem);
            } else {
                cerr << "Unknown planner type: " << planner_type_ << endl;
                exit(EXIT_FAILURE);
            }
    
            // cout << "Plan:" << endl;
            auto plan = make_shared<pddlboat::Plan>(subproblem);
            if (!task_planner->solve(*plan))
            {
                cerr << "Failed to solve!" << endl;
                // exit(1);
                regionSubproblems.erase(curr_dfa_state);
                continue;
            } else {
                cout << "Solved the subproblem: " << *plan << endl;
            }

            // Get a trace of states from the plan.
            auto state_trace = plan->getStepStates(true);
            vector<ProductState> product_path;

            if (state_trace.empty()) {
                cerr << "ERROR: the plan is empty!" << endl;
                regionSubproblems.clear();
                continue;
            }
            
            // Handle the last domain state separately
            last_domain_state = make_shared<PDDLState>(state_trace.back());
            next_prod_state = ProductState(last_domain_state, next_dfa_state);

            // Check if it was already explored.
            if (visited.find(next_prod_state) != visited.end()) {
                // Skip this state.
                // TODO: modify the problem to avoid this domain state?
                num_repeated_paths++;
                // cout << "ERROR: Found a previously explored path to the dfa state " << next_dfa_state << endl;
                if (num_repeated_paths >= max_num_repeated_paths) {
                    // cerr << "Failed to find the alternative path!" << endl;
                    regionSubproblems.clear();
                    // regionSubproblems.erase(curr_dfa_state);
                    // num_repeated_paths = 0;
                }
                continue;
            }
            // Visit the state.
            visited.insert(next_prod_state);

            // Get a bdd corresponding to last PDDL state.
            bdd last_domain_state_bdd = domain_manager_->get_state_bdd(last_domain_state);
            // Get a bdd corresponding to the transition in the DFA.
            bdd next_edge_cond = dfa_nodes.at(currentRegionIndex+1)->getParentEdgeCondition();
            
            // cout << "Edge condition for " << curr_dfa_state << "->" << next_dfa_state << " is as follows: " << endl;
            // print_bdd(next_edge_cond);

            if (!dfa_manager_->is_transition_valid(next_edge_cond, last_domain_state_bdd)) {
                // cout << "ERROR: Transition doesn't satisfy an edge condition for " << curr_dfa_state << "->" << next_dfa_state << endl;
                continue;
            }

            // cout << "Subproblem has a state trace of length = " << state_trace.size() << endl;
            bdd self_edge_cond = dfa_nodes.at(currentRegionIndex)->getSelfEdgeCondition();
            // cout << "Self-edge condition for " << curr_dfa_state << "->" << curr_dfa_state << " is as follows: " << endl;
            // print_bdd(self_edge_cond);
            bool valid_dfa_state = true;
            // Iterate over all domain states except the last one
            for (int i = state_trace.size() - 2; i >= 0; --i) {
                // Create a PDDLState
                auto domain_state = make_shared<PDDLState>(state_trace.at(i));
                // Get a bdd corresponding to this PDDL state.
                bdd domain_state_bdd = domain_manager_->get_state_bdd(domain_state);
                // cout << "Domain state: " << endl << *domain_state << "has a bdd as follows: " << domain_state_bdd << endl;
                // Verify that domain states correspond to the current dfa state.
                if (!((i == 0) || (self_edge_cond == bddtrue) || (dfa_manager_->is_transition_valid(self_edge_cond, domain_state_bdd)))) {
                    // cout << "ERROR: Transition doesn't satisfy a self-edge condition of dfa state = " << curr_dfa_state << endl;
                    valid_dfa_state = false;
                }
                product_path.emplace_back(domain_state, curr_dfa_state);
            }

            // Implement max threshold for the number of wrong transitions.
            if (!valid_dfa_state) {
                num_wrong_dfa_trans++;
                // cout << "ERROR: Solution path goes beyond the dfa state = " << curr_dfa_state << endl;
                if (num_wrong_dfa_trans >= max_num_wrong_dfa_trans) {
                    cerr << "Failed to solve within the same dfa region!" << endl;
                    regionSubproblems.erase(curr_dfa_state);
                }
                continue;
            }

            // We were able to get to the next DFA state in a trace!
            // cout << "Succesfully realized this transition: " << curr_dfa_state << "=>" << next_dfa_state << endl;

            num_wrong_dfa_trans = 0;
            num_repeated_paths = 0;

            // Add it to the parent map to be able to backtrack.
            parent_map.emplace(next_prod_state, product_path);
            // cout << "emplaced state: " << next_state << endl;

            // Make all states in the path visited.
            // for (const auto& product_state : product_path) {
            //     visited.insert(product_state);
            // }

            // Update the cost to 0 based on the success of the transition.
            dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(currentRegionIndex + 1), SUCCESS_COST);

            // Optimization 1: cache paths that cross the dfa states
            if (cache_) {
                vector<ProductState> path_to_cache_reversed = construct_path(parent_map, next_prod_state, true, curr_dfa_state);

                // Now we create a transition and add it to a product graph.
                // Serves as a "skip-connection".
                product_manager_->cache_path(path_to_cache_reversed, next_edge_cond);

                // cout << "Cached this transition: " << curr_dfa_state << "=>" << next_dfa_state << endl;

                // TODO: cache it as a path between modalities in DomainManager (can be reused for other TEG problems).
            }
        }

        currentRegionIndex++;

        // Remember the last dfa state we were able to reach.
        if (maxRegionIndexReached < currentRegionIndex) {
            maxRegionIndexReached = currentRegionIndex;
        }
        // Check if it's the accepting state (last in the dfa trace).
        // If so, we have a solution!
        if (currentRegionIndex == dfa_trace.size() - 1) {
            // cout << "We have reached the last accepting state in the dfa trace" << endl;
            // Backtrack to get the full path.
            product_path_ = construct_path(parent_map, next_prod_state);
            save_paths();
            return;
        }

        // Instantiate a subproblem.
        pddlboat::ProblemPtr next_subproblem = create_subproblem(dfa_nodes.at(currentRegionIndex+1)->getParentEdgeCondition(), last_domain_state);
        regionSubproblems.insert({next_dfa_state, next_subproblem});
        // cout << "Printing a map after inserting a problem for dfa_state=" <<  next_dfa_state << endl;
        // for (const auto& pair : regionSubproblems) {
        //     cout << "(DFA = " << pair.first << ", goal = ";
        //     pair.second->goal->toPDDL(cout) << ")" << endl;
        // }
    } 
    // We failed to realize the provided dfa trace.
    // Update the cost for the dfa transition we couldn't realize.
    dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(maxRegionIndexReached + 1), FAILURE_COST);
}

vector<ProductState> PDDLProblem::construct_path(const map<ProductState, vector<ProductState>>& parent_map, ProductState target_state, bool cached, size_t start_dfa_state) {
    vector<ProductState> path;
    path.push_back(target_state);

    while (parent_map.find(target_state) != parent_map.end()) {

        // cout << "Target state was found: " << target_state << endl;

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

    // cout << "Target state was NOT found: " << target_state << endl;

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

double PDDLProblem::get_dfa_construction_time() const {
    return dfa_construction_time_.count();
}

size_t PDDLProblem::get_num_generated_nodes() const {
    return product_manager_->get_num_expanded_nodes();
}
size_t PDDLProblem::get_num_expanded_nodes() const {
    return num_expanded_nodes_;
}

size_t PDDLProblem::get_num_of_backtracks() const {
    return num_of_backtracks_;
}

pddlboat::ProblemPtr PDDLProblem::create_subproblem(bdd& edge_cond, shared_ptr<PDDLState> start_state) {
    // Collect bound predicates
    auto bound_predicates = collect_bound_predicates(edge_cond);

    // Separate constraints and goals based on start state
    vector<pddlboat::ExpressionPtr> constraints;
    vector<pddlboat::ExpressionPtr> goals;
    split_constraints_and_goals(bound_predicates, start_state->getPddlboatStatePtr(), constraints, goals);

    // Step 1: create a domain with constraints for the subproblem.
    // Create a derived predicate for constraints that must always hold true.
    auto constraints_expr = pddlboat::makeAnd(constraints);

    // Update a domain by adding constraints
    auto updated_domain = get_domain_with_constraints(constraints_expr, domain_->getPddlboatDomainPtr());

    // Step 2: Use the new domain to create a new subproblem
    auto subproblem = make_shared<pddlboat::Problem>(pddlProblem_->name, updated_domain);

    // Step 3: Set the problem objects.
    subproblem->objects = pddlProblem_->objects;

    // Step 4: Set the start state
    subproblem->start = start_state->getPddlboatStatePtr();

    // Step 5: Create a new goal with the unsatisfied predicates
    subproblem->goal = pddlboat::makeAnd(goals);

    auto domain = subproblem->domain;
    cout << "Subproblem's domain is: " << endl;
    domain->toPDDL(std::cout) << std::endl;

    cout << "Subgoal is: " << endl;
    subproblem->goal->toPDDL(std::cout) << std::endl;

    return subproblem;
}

map<pddlboat::PredicatePtr, bool> PDDLProblem::collect_bound_predicates(bdd& edge_cond) {
    // Vector to collect bound predicates (as a map from PredicatePtrs to a boolean)
    map<pddlboat::PredicatePtr, bool> bound_predicates;

    for (size_t i = 0; i < bdd_dict_->var_map.size() + 1; ++i) {
        bdd p = bdd_ithvar(i);  // Represents the atomic proposition 'p'
        bdd np = bdd_nithvar(i); // Represents the negation of 'p'
        if ((bdd_restrict(edge_cond, np) != bddfalse) && (bdd_restrict(edge_cond, p) != bddfalse)) {
            // cout << "Continue: var not there" << endl;
            continue;
        }
        // Get the atomic proposition name
        string ap_name = domain_manager_->get_ap_name(i);

        // Split the atomic proposition name by "_"
        istringstream ss(ap_name);
        string token;
        vector<string> tokens;
        while (getline(ss, token, '_')) {
            tokens.push_back(token);
        }

        if (tokens.size() < 2) {
            cerr << "Invalid AP name format: " << ap_name << endl;
            // continue;
        }

        size_t token_num = 1;
        // size_t last_valid_token_num = 0;
        // The first part is the name of the predicate
        string predicate_name = tokens.at(0);
        // cout << "Printing predicates" << endl;
        // for (const auto& pred_pair : pddlProblem_->domain->predicates) {
        //     cout << pred_pair.first << endl;
        // }
        // cout << "=====" << endl;

        // Find the predicate definition
        auto predicate_it = pddlProblem_->domain->predicates.find(predicate_name);
        // auto last_valid_predicate_it = pddlProblem_->domain->predicates.end();
        // if (predicate_it != pddlProblem_->domain->predicates.end()) {
        //     last_valid_predicate_it = predicate_it;
        //     last_valid_token_num = token_num;
        // }
        // // Needed for rovers
        // while (token_num < tokens.size()) {
        //     predicate_name = predicate_name + "_" + tokens.at(token_num);
        //     predicate_it = pddlProblem_->domain->predicates.find(predicate_name);
        //     token_num++;
        //     if (predicate_it != pddlProblem_->domain->predicates.end()) {
        //         last_valid_predicate_it = predicate_it;
        //         last_valid_token_num = token_num;
        //     }
        // }
        if (predicate_it == pddlProblem_->domain->predicates.end()) {
            // Needed for openstacks
            while (token_num < tokens.size() && predicate_it == pddlProblem_->domain->predicates.end()) {
                predicate_name = predicate_name + "-" + tokens.at(token_num);
                predicate_it = pddlProblem_->domain->predicates.find(predicate_name);
                token_num++;
            }
        }
        // predicate_it = last_valid_predicate_it;
        // token_num = last_valid_token_num;

        if (predicate_it == pddlProblem_->domain->predicates.end()) {
            std::cerr << "Predicate not found: " << predicate_name << std::endl;
            continue;
        }

        auto p_predicate = predicate_it->second;

        vector<string> bindings(tokens.begin() + token_num, tokens.end());
        // Bind the predicate
        auto bound_pred = p_predicate->bind(bindings, pddlProblem_);

        // Check if including the proposition (or its negation) makes a difference in the XOR result
        if (bdd_restrict(edge_cond, np) == bddfalse) {
            // Condition contains positive p
            // std::cout << "Found a positive ap: " << ap_name << std::endl;
            bound_predicates[bound_pred] = true;
        } else if (bdd_restrict(edge_cond, p) == bddfalse) {
            // Condition contains negative p
            // std::cout << "Found a negative ap: " << ap_name << std::endl;
            // Negate it
            bound_predicates[bound_pred] = false;
        }
    }
    return bound_predicates;
}

void PDDLProblem::split_constraints_and_goals(
    const map<pddlboat::PredicatePtr, bool>& bound_predicates,
    const pddlboat::StatePtr& start_state,
    vector<pddlboat::ExpressionPtr>& constraints,
    vector<pddlboat::ExpressionPtr>& goals) {
    for (const auto& pred_pair : bound_predicates) {
        // Get the predicate and its associated boolean value 
        // (true for positive, false for negated)
        auto predicate = pred_pair.first;
        bool is_positive = pred_pair.second;

        // Create a literal according to its sign.
        pddlboat::ExpressionPtr signed_predicate = predicate; 
        // Negate the predicate if needed.
        if (!is_positive) {
            signed_predicate = pddlboat::makeNot(predicate);
        }

        // Check if the predicate (or its negation) is true in the start state.
        if (predicateHoldsInState(start_state, predicate) == is_positive) {
            std::cout << "Predicate is satisfied in the start_state: ";
            signed_predicate->toPDDL(std::cout) << std::endl;
            // The start state satisfies this predicate, so it's a constraint.
            constraints.push_back(signed_predicate);
        } else {
            std::cout << "Predicate is NOT satisfied in the start_state: ";
            signed_predicate->toPDDL(std::cout) << std::endl;
            // The start state does not satisfy this predicate, so it's a goal.
            goals.push_back(signed_predicate);
        }
    }
}

pddlboat::DomainPtr PDDLProblem::get_domain_with_constraints(pddlboat::ExpressionPtr constraints_expr, const pddlboat::DomainPtr& original_domain) {
    // Create a deep copy of the original domain
    pddlboat::DomainPtr domain = make_shared<pddlboat::Domain>(*original_domain);

    string constraint_predicate_name = "constraints";

    // Check if the derived predicate already exists
    if (domain->isDerivedPredicate(constraint_predicate_name)) {
        std::cerr << "ERROR: Derived predicate 'constraints' already exists in the domain." << std::endl;
        return original_domain;
    }

    // Add the derived predicate to the domain
    // No variables for the constraints predicate
    pddlboat::VariableList empty_vars;
    auto derived_predicate_definition = pddlboat::makeDerivedPred(constraint_predicate_name, empty_vars, constraints_expr);

    domain->addDerivedPredicate(derived_predicate_definition);

    cout << "Added derived predicate: " << constraint_predicate_name << endl;

    // Clear all actions in the new domain.
    domain->actions.clear();

    // Update all action preconditions to include (constraints)
    for (const auto& action_pair : original_domain->actions) {
        auto original_action = action_pair.second;

        // Bind the 'constraints' predicate for precondition use
        auto constraint_predicate = derived_predicate_definition->bind();

        // Create a new precondition.
        auto updated_precondition = pddlboat::makeAnd({original_action->precondition, constraint_predicate});

        // Create a new action with the updated precondition
        auto new_action = pddlboat::makeAction(
            original_action->name, 
            original_action->parameters, 
            updated_precondition, 
            original_action->effect
        );

        // Add the new action to the copied domain
        domain->addAction(new_action);
    }
    return domain;
}

bool PDDLProblem::predicateHoldsInState(const pddlboat::StatePtr& state, const pddlboat::PredicatePtr& predicate) {
    // Retrieve the predicate name and arguments from the predicate object.
    string predicate_name = predicate->name();

    // Get the arguments (bindings) of the predicate
    vector<string> args;
    for (const auto &binding : predicate->bindings) {
        args.push_back(binding.name); 
    }
    // Check if the predicate is true in the state
    return state->isTrue(predicate_name, args);
}


void PDDLProblem::print_bdd(bdd& expr) {
    cout << "Printing bdd = " << expr << endl;
    for (size_t i = 0; i < bdd_dict_->var_map.size() + 1; ++i) {
        bdd p = bdd_ithvar(i);  // Represents the atomic proposition 'p'
        bdd np = bdd_nithvar(i); // Represents the negation of 'p'
        if ((bdd_restrict(expr, np) != bddfalse) && (bdd_restrict(expr, p) != bddfalse)) {
            // cout << "Continue: var not there" << endl;
            continue;
        }
        // Get the atomic proposition name
        string ap_name = domain_manager_->get_ap_name(i);

        // Check if including the proposition (or its negation) makes a difference in the XOR result
        if (bdd_restrict(expr, np) == bddfalse) {
            // Condition contains positive p
            std::cout << "(" << ap_name << "), ";

        } else if (bdd_restrict(expr, p) == bddfalse) {
            // Condition contains negative p: an obstacle
            std::cout << "(not " << ap_name << "), ";
        }
    }
}

std::string PDDLProblem::write_solution_to_file() const {
    std::string dir_name = "solutions";
    dir_name += "_with_" + planner_type_;
    // Create the solutions directory if it doesn't exist
    std::filesystem::create_directory(dir_name);

    // Create the domain subdirectory if it doesn't exist
    std::string domain_dir = dir_name + "/" + domain_name_;
    std::filesystem::create_directory(domain_dir);

    // Construct the file path
    std::string file_path = domain_dir + "/" + problem_name_ + "_solution.txt";

    // Open the file
    std::ofstream ofs(file_path);

    // Check if the file is open
    if (!ofs.is_open()) {
        std::cerr << "Failed to open file: " << file_path << std::endl;
        return "";
    }

    // Write the DFA path to the file
    ofs << "DFA Path:" << std::endl;
    if (!dfa_path_.empty()) {
        ofs << dfa_path_.front();
        for (auto it = std::next(dfa_path_.begin()); it != dfa_path_.end(); ++it) {
            ofs << " -> " << *it;
        }
    }
    ofs << std::endl << std::endl;

    // Write the product path to the file
    ofs << "Product Path:" << std::endl;
    if (!product_path_.empty()) {
        ofs << product_path_.front();
        for (auto it = std::next(product_path_.begin()); it != product_path_.end(); ++it) {
            ofs << " -> " << *it;
        }
    }
    ofs << std::endl;

    // Close the file
    ofs.close();

    std::cout << "Solution written to " << file_path << std::endl;
    return file_path;
}

void PDDLProblem::extract_names(const std::string& problemFile) {
    // Extracting the problem name
    size_t lastSlashPos = problemFile.find_last_of('/');
    size_t extensionPos = problemFile.find(".pddl");
    if (lastSlashPos != std::string::npos && extensionPos != std::string::npos) {
        problem_name_ = problemFile.substr(lastSlashPos + 1, extensionPos - lastSlashPos - 1);
    } else {
        std::cerr << "Invalid problem file path format." << std::endl;
        problem_name_ = "problem"; // Default name
    }

    // Extracting the domain name
    if (lastSlashPos != std::string::npos) {
        size_t secondLastSlashPos = problemFile.find_last_of('/', lastSlashPos - 1);
        if (secondLastSlashPos != std::string::npos) {
            domain_name_ = problemFile.substr(secondLastSlashPos + 1, lastSlashPos - secondLastSlashPos - 1);
        } else {
            std::cerr << "Invalid problem file path format for domain name." << std::endl;
            domain_name_ = "domain"; // Default name
        }
    } else {
        std::cerr << "Invalid problem file path format." << std::endl;
        domain_name_ = "domain"; // Default name
    }
    
    // Debug output
    // std::cout << "Problem Name: " << problem_name_ << std::endl;
    // std::cout << "Domain Name: " << domain_name_ << std::endl;
}