#include "PDDLProblem.h"
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>
#include <iostream>
#include <sstream>
#include <cassert>
#include <fstream>
#include <filesystem>  // Requires C++17

PDDLProblem::PDDLProblem(const string& problemFile, shared_ptr<PDDLDomain> domainPtr, const string& planner_type, const string& search_type, const string& name_connector, bool cache, bool feedback, bool use_landmarks, bool hamming_dist, bool save_dfa, size_t subproblem_timeout, int backtracks_limit, size_t max_num_wrong_dfa_trans) 
: domain_(domainPtr), planner_type_(planner_type), search_type_(search_type),
name_connector_(name_connector), cache_(cache), feedback_(feedback),
use_landmarks_(use_landmarks), hamming_dist_(hamming_dist), save_dfa_(save_dfa), bdd_dict_(make_shared<spot::bdd_dict>()), num_expanded_nodes_(0), subproblem_timeout_(subproblem_timeout), backtracks_limit_(backtracks_limit), max_num_wrong_dfa_trans_(max_num_wrong_dfa_trans) {

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
    std::cout << "Time for constructing an automaton: " << dfa_construction_time_.count() << " seconds" << std::endl;

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
    // Clear any existing paths from previous executions.
    product_path_.clear();
    dfa_path_.clear();

    // Retrieve the start state of the DFA.
    size_t dfa_start_state = dfa_manager_->get_start_state();
    // Error if the DFA is trivial (only a single accepting state).
    if (dfa_manager_->is_accepting_state(dfa_start_state)) {
        cerr << "ERROR: DFA has a single state. LTL formula is not descriptive enough." << endl;
        return {};
    }

    // Initializethe priority queue for the DFA paths from the start DFA state.
    dfa_manager_->initialize_node_priority_queue();
    num_of_backtracks_ = -1;

    // Backtracking loop to find a valid product path.
    while (product_path_.empty() && num_of_backtracks_ < backtracks_limit_) {
        num_of_backtracks_++;
        // Generate a likely DFA trace that leads to an acceptance state.
        auto end_trace_node = dfa_manager_->generate_dfa_path();
        // If no more DFA traces are available, exit the loop.
        if (!end_trace_node) {
            break;
        }

        // Attempt to realize the current DFA trace in the task domain.
        realize_dfa_trace(end_trace_node);
    }
    // Return the product path, or an empty vector if no solution was found.
    return product_path_;
}

void PDDLProblem::realize_dfa_trace(shared_ptr<DFANode>& end_trace_node) {

    if (planner_type_ != "manual") {
        // 1. Create an instanse of the subproblem 
        // 2. Call the off-the-shelf planner
        realize_dfa_trace_with_planner(end_trace_node);
    } else {
        realize_dfa_trace_manually(end_trace_node);
    }
}

void PDDLProblem::realize_dfa_trace_manually(shared_ptr<DFANode>& end_trace_node) {

    auto dfa_trace = end_trace_node->getPathFromRoot();
    auto dfa_nodes = end_trace_node->getNodePathFromRoot();

    if (!validate_dfa_trace(dfa_trace)) return;

    size_t dfa_start_state = dfa_trace.front();
    ProductState start_state(start_domain_state_, dfa_start_state);

    size_t currentRegionIndex = 0;
    size_t maxRegionIndexReached = 0;

    set<ProductState> visited;
    visited.insert(start_state);

    // Initialize a map of priority queues 
    // (one queue for each DFA state in the trace)
    map<size_t, priority_queue<pair<int, ProductState>, vector<pair<int, ProductState>>, greater<pair<int, ProductState>>>> regionQueues;
    map<ProductState, vector<ProductState>> parent_map;
    map<size_t, DomainStateSet> landmarks; 

    int curr_state_heuristic = 0;
    if (use_landmarks_) {
        auto next_dfa_edge_condition = dfa_nodes.at(1)->getParentEdgeCondition();
        landmarks[dfa_start_state] = product_manager_->sample_landmarks(next_dfa_edge_condition, start_domain_state_);
        curr_state_heuristic = compute_heuristics(start_state, landmarks, dfa_start_state);
    }

    enqueue_product_state(regionQueues, start_state, curr_state_heuristic, dfa_start_state);

    size_t curr_num_wrong_dfa_trans = 0;

    while (!regionQueues[dfa_start_state].empty()) {
        size_t curr_dfa_state = dfa_trace.at(currentRegionIndex);    
        auto& queue = regionQueues[curr_dfa_state];

        if (queue.empty()) {
            // Backtrack if necessary
            assert(currentRegionIndex > 0 && "currentRegionIndex must be positive");
            currentRegionIndex--;
            continue;
        }
        // if (queue.size() == 1) {
        //     cout << "Trying to realize this transition: " << curr_dfa_state << "=>" << dfa_trace.at(currentRegionIndex+1) << endl;
        // }
        // Queue is not empty!
        auto current_state_pair = queue.top();
        ProductState current_state = current_state_pair.second;
        queue.pop();

        // Check if state was expanded?
        // If not, then generate successors for this state!
        if (product_manager_->state_not_expanded(current_state)) {
            // Expand the state.
            num_expanded_nodes_++;
            product_manager_->generate_successors(current_state);
        }

        for (auto& transition : product_manager_->get_transitions(current_state)) {
            // Get the product state where this transition leads. 
            ProductState next_state = transition.out_state();
            // Check if it was already explored.
            if (visited.find(next_state) != visited.end()) continue;

            // Get the next dfa state.
            size_t next_dfa_state = next_state.get_dfa_state();

            // We are interested only in the transitions that either stay in the same dfa state or lead to the next dfa state in the trace.
            if (next_dfa_state != dfa_trace.at(currentRegionIndex) &&
            next_dfa_state != dfa_trace.at(currentRegionIndex + 1)) {
                // Skip states that lead to any other dfa states.
                // Update cost for this transition to 0.
                if (feedback_ && dfa_manager_->dfa_transition_cost(curr_dfa_state, next_dfa_state) != SUCCESS_COST) {
                    dfa_manager_->update_dfa_transition_cost(curr_dfa_state, next_dfa_state, SUCCESS_COST);
                    // Cache this for future reuse.
                    // TODO: implement caching!
                    // if (cache_ && !transition.isCached()) {
                    //     vector<ProductState> path_to_cache_reversed = construct_path(parent_map, next_state, true, curr_dfa_state);
                    //     // Now we create a transition and add it to a product graph.
                    //     // Serves as a "skip-connection".
                    //     product_manager_->cache_path(path_to_cache_reversed, transition.dfa_edge_condition());
                    // }
                }
                if (!handle_wrong_transitions(curr_num_wrong_dfa_trans, dfa_nodes.back(), dfa_nodes.at(currentRegionIndex + 1))) {
                    return;
                }
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

                // TODO: debug landmarks approach later.
                if (use_landmarks_ && landmarks.count(next_dfa_state) == 0) {
                    auto next_dfa_edge_condition = dfa_nodes.at(currentRegionIndex + 1)->getParentEdgeCondition();
                    landmarks[next_dfa_state] = product_manager_->sample_landmarks(next_dfa_edge_condition, next_state.get_domain_state());
                }
            } 
            // Compute the heuristic score for this state and push it into the queue.
            int next_state_cost = current_state_pair.first + DEFAULT_COST; 
            if (use_landmarks_) {
                int curr_heuristic_cost = compute_heuristics(current_state, landmarks, curr_dfa_state);
                int next_heuristic_cost = compute_heuristics(next_state, landmarks, next_dfa_state);
                next_state_cost += (next_heuristic_cost - curr_heuristic_cost);
            }

            // Push this new product state onto the corresponding queue.
            enqueue_product_state(regionQueues, next_state, next_state_cost, next_dfa_state);

            // This state has been visited now!
            visited.insert(next_state);
        }
    }
    // We failed to realize the provided dfa trace.
    // Update the cost for the dfa transition we couldn't realize.
    dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(maxRegionIndexReached + 1), FAILURE_COST);
}

void PDDLProblem::realize_dfa_trace_with_planner(shared_ptr<DFANode>& end_trace_node) {

    auto dfa_trace = end_trace_node->getPathFromRoot();
    auto dfa_nodes = end_trace_node->getNodePathFromRoot();

    if (!validate_dfa_trace(dfa_trace)) return;

    size_t currentRegionIndex = 0;
    size_t maxRegionIndexReached = 0;
    int max_num_wrong_dfa_trans = 3;
    int max_num_repeated_paths = 3;
    int num_wrong_dfa_trans = 0;
    int num_repeated_paths = 0;

    set<ProductState> visited;
    map<size_t, pddlboat::ProblemPtr> regionSubproblems;
    map<ProductState, vector<ProductState>> parent_map;

    size_t dfa_start_state = dfa_trace.front();
    ProductState start_state(start_domain_state_, dfa_start_state);
    visited.insert(start_state);

    // Instantiate a subproblem for the first transition
    pddlboat::ProblemPtr start_subproblem = create_subproblem(dfa_nodes.at(1)->getParentEdgeCondition(), start_domain_state_);
    regionSubproblems.insert({dfa_start_state, start_subproblem});
    
    while (!regionSubproblems.empty()) {
        size_t curr_dfa_state = dfa_trace.at(currentRegionIndex);
        size_t next_dfa_state = dfa_trace.at(currentRegionIndex + 1);
        // cout << "Trying to realize a transition: " << curr_dfa_state << "=>" << next_dfa_state << endl;
        // Check if there is a subproblem for the current DFA state
        if (regionSubproblems.count(curr_dfa_state) == 0) {
            // Backtrack if necessary
            assert(currentRegionIndex > 0 && "currentRegionIndex must be positive");
            currentRegionIndex--;
            continue;  
        }

        auto& subproblem = regionSubproblems[curr_dfa_state];
        // std::cout << "Solving subproblem for DFA state: " << curr_dfa_state << std::endl;
        // subproblem->goal->toPDDL(std::cout) << std::endl;

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
                    // Cached solution found
                    cout << "WARNING: Using a cached solution!" << endl;
                    // Add it to the parent map to be able to backtrack.
                    parent_map.emplace(next_prod_state, transition.path());
                    last_domain_state = static_pointer_cast<PDDLState>(next_prod_state.get_domain_state());
                    retrieved_path = true;
                }
            }
        }

        if (!retrieved_path) {
            // Use the planner to solve the subproblem
            // Instantiate the planner based on the type
            shared_ptr<pddlboat::Planner> task_planner = get_task_planner(subproblem);
            auto plan = make_shared<pddlboat::Plan>(subproblem);
            if (!task_planner->solve(*plan, subproblem_timeout_))
            {
                regionSubproblems.erase(curr_dfa_state);
                continue;
            }

            // Retrieve the plan's steps (which includes actions and states)
            auto plan_steps = plan->getSteps();
            if (plan_steps.empty()) {
                cerr << "ERROR: the plan is empty! Search failed." << endl;
                regionSubproblems.clear(); // Finish the search
                return;
            }


            // // Get a trace of states from the plan.
            // auto state_trace = plan->getStepStates(true);
            
            // Get the final state from the plan and check if it satisfies the DFA edge condition
            // last_domain_state = make_shared<PDDLState>(state_trace.back());
            last_domain_state = make_shared<PDDLState>(plan_steps.back().state->clone());
            next_prod_state = ProductState(last_domain_state, next_dfa_state);

            // Check if it was already explored.
            if (visited.find(next_prod_state) != visited.end()) {
                // Skip this state.
                // TODO: modify the problem to avoid this domain state?
                num_repeated_paths++;
                // cout << "ERROR: Found a previously explored path to the dfa state " << next_dfa_state << endl;
                if (num_repeated_paths >= max_num_repeated_paths) {
                    // cerr << "Failed to find the alternative path! Search failed." << endl;
                    regionSubproblems.clear(); // Finish the search
                }
                continue;
            }
            // Visit the state.
            visited.insert(next_prod_state);

            // Get a bdd corresponding to last PDDL state.
            bdd last_domain_state_bdd = domain_manager_->get_state_bdd(last_domain_state);
            // Get a bdd corresponding to the transition in the DFA.
            bdd next_edge_cond = dfa_nodes.at(currentRegionIndex+1)->getParentEdgeCondition();

            if (!dfa_manager_->is_transition_valid(next_edge_cond, last_domain_state_bdd)) {
                // cout << "ERROR: Transition doesn't satisfy an edge condition for " << curr_dfa_state << "->" << next_dfa_state << endl;
                continue;
            }

            bool valid_dfa_state = true;
            vector<ProductState> product_path;
            // for (const auto& step : plan->getSteps()) {
            //     // Assuming you are iterating over plan steps
            //     ActionPtr current_action = step.action;
            //     Assignment current_assignment = step.assignment;

            //     // Create a ProductState with the action and store it
            //     next_prod_state = ProductState(last_domain_state, next_dfa_state, current_action);

            //     // Store the action in the parent map for tracing back later
            //     parent_map.emplace(next_prod_state, product_path);
            // }
            // Iterate over all domain states except the last one
            // for (int i = state_trace.size() - 2; i >= 0; --i) {
            //     // Create a PDDLState
            //     auto domain_state = make_shared<PDDLState>(state_trace.at(i));
            //     // Get a bdd corresponding to this PDDL state.
            //     bdd domain_state_bdd = domain_manager_->get_state_bdd(domain_state);
    
            //     // Verify that domain states correspond to the current dfa state.
            //     if (!((i == 0) || dfa_manager_->is_transition_valid(dfa_nodes.at(currentRegionIndex)->getSelfEdgeCondition(), domain_state_bdd))) {
            //         // cout << "ERROR: Transition doesn't satisfy a self-edge condition of dfa state = " << curr_dfa_state << endl;
            //         valid_dfa_state = false;
            //     }
            //     product_path.emplace_back(domain_state, curr_dfa_state);
            // }

            // Iterate over the steps to create corresponding ProductStates
            for (int i = plan_steps.size() - 1; i >= 0; --i) {
                // Extract the current step and wrap it in a shared pointer
                auto current_step = make_shared<pddlboat::Plan::Step>(plan_steps.at(i));

                // Create a PDDLState from the prior state of the step
                auto domain_state = make_shared<PDDLState>(current_step->prior->clone());

                // Get a BDD corresponding to this PDDL state
                bdd domain_state_bdd = domain_manager_->get_state_bdd(domain_state);

                // Verify that domain states correspond to the current DFA state
                if (!((i == 0) || dfa_manager_->is_transition_valid(dfa_nodes.at(currentRegionIndex)->getSelfEdgeCondition(), domain_state_bdd))) {
                    valid_dfa_state = false;
                }

                // Create a ProductState with the Plan::Step and push it to the product_path_
                product_path.emplace_back(domain_state, curr_dfa_state, current_step);
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

            // Update the cost to 0 based on the success of the transition.
            dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(currentRegionIndex + 1), SUCCESS_COST);

            // Optimization 1: cache paths that cross the dfa states
            if (cache_) {
                vector<ProductState> path_to_cache_reversed = construct_path(parent_map, next_prod_state, true, curr_dfa_state);

                // Now we create a transition and add it to a product graph.
                // Serves as a "skip-connection".
                product_manager_->cache_path(path_to_cache_reversed, next_edge_cond);
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
            // Backtrack to get the full path.
            product_path_ = construct_path(parent_map, next_prod_state);
            save_paths();
            return;
        }

        // Instantiate a subproblem.
        pddlboat::ProblemPtr next_subproblem = create_subproblem(dfa_nodes.at(currentRegionIndex+1)->getParentEdgeCondition(), last_domain_state);
        regionSubproblems.insert({next_dfa_state, next_subproblem});
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

size_t PDDLProblem::get_plan_length() const {
    if (domain_path_.empty()) {
        return 0;
    }
    return domain_path_.size() - 1;
}

pddlboat::ProblemPtr PDDLProblem::create_subproblem(bdd& edge_cond, shared_ptr<PDDLState> start_state) {

    // Collect bound predicates 
    auto bound_predicates = collect_bound_predicates(edge_cond);
    // If there are no predicates whose restriction makes the condition false, we have a disjunction!
    bool is_disjunction = bound_predicates.empty();

    // Separate constraints and goals based on start state
    vector<pddlboat::ExpressionPtr> constraints;
    vector<pddlboat::ExpressionPtr> goals;

    pddlboat::DomainPtr domain = domain_->getPddlboatDomainPtr();

    // Step 1: get goals and create a domain with constraints.
    if (is_disjunction) {
        // Collect the predicates that are part of the disjunction
        bound_predicates = collect_bound_predicates_in_OR(edge_cond);
        // For disjunction, we don't need constraints.
        get_goals(bound_predicates, goals);
    } else {
        // For conjunction.
        split_constraints_and_goals(bound_predicates, start_state->getPddlboatStatePtr(), constraints, goals);

        // Create a derived predicate for constraints that must always hold true.
        auto constraints_expr = pddlboat::makeAnd(constraints);

        // Update a domain by adding constraints
        domain = get_domain_with_constraints(constraints_expr, domain);
    }

    // Step 2: Use the new domain to create a new subproblem
    auto subproblem = make_shared<pddlboat::Problem>(pddlProblem_->name, domain);

    // Step 3: Set the problem objects.
    subproblem->objects = pddlProblem_->objects;

    // Step 4: Set the start state
    subproblem->start = start_state->getPddlboatStatePtr();

    // Step 5: Create a new goal
    if (is_disjunction) {
        subproblem->goal = pddlboat::makeOr(goals);
    } else {
        subproblem->goal = pddlboat::makeAnd(goals);
    }

    // auto domain = subproblem->domain;
    // cout << "Subproblem's domain is: " << endl;
    // domain->toPDDL(std::cout) << std::endl;

    // cout << "Subgoal is: " << endl;
    // subproblem->goal->toPDDL(std::cout) << std::endl;

    return subproblem;
}

pddlboat::PredicatePtr PDDLProblem::extract_predicate_from_ap_name(const std::string& ap_name) {
    // Split the atomic proposition name by "_"
    vector<string> tokens;
    size_t start = 0, end = 0;
    while ((end = ap_name.find('_', start)) != string::npos) {
        tokens.push_back(ap_name.substr(start, end - start));
        start = end + 1;
    }
    tokens.push_back(ap_name.substr(start));

    if (tokens.size() < 2) {
        cerr << "Invalid AP name format: " << ap_name << endl;
        return nullptr;  // Return null if the format is invalid
    }

    string predicate_name = tokens.at(0);
    auto predicate_it = pddlProblem_->domain->predicates.find(predicate_name);
    auto last_valid_predicate_it = predicate_it;
    size_t last_valid_token_num = 0;

    // This loop tries to progressively add tokens to find a valid predicate
    for (size_t token_num = 1; token_num < tokens.size(); ++token_num) {
        predicate_name = predicate_name + name_connector_ + tokens.at(token_num);
        predicate_it = pddlProblem_->domain->predicates.find(predicate_name);
        if (predicate_it != pddlProblem_->domain->predicates.end()) {
            last_valid_predicate_it = predicate_it;
            last_valid_token_num = token_num;
        }
    }

    if (last_valid_predicate_it == pddlProblem_->domain->predicates.end()) {
        cerr << "Predicate not found for AP: " << ap_name << endl;
        return nullptr;  // Return null if the predicate is not found
    }

    auto p_predicate = last_valid_predicate_it->second;

    vector<string> bindings(tokens.begin() + last_valid_token_num + 1, tokens.end());
    return p_predicate->bind(bindings, pddlProblem_);  // Return the bound predicate
}



map<pddlboat::PredicatePtr, bool> PDDLProblem::collect_bound_predicates(bdd& edge_cond) {
    // Vector to collect bound predicates (as a map from PredicatePtrs to a boolean)
    map<pddlboat::PredicatePtr, bool> bound_predicates;

    // cout << "Printing predicates" << endl;
    // for (const auto& pred_pair : pddlProblem_->domain->predicates) {
    //     cout << pred_pair.first << endl;
    // }
    // cout << "=====" << endl;

    for (size_t i = 0; i < bdd_dict_->var_map.size() + 1; ++i) {
        bdd p = bdd_ithvar(i);  // Represents the atomic proposition 'p'
        bdd np = bdd_nithvar(i); // Represents the negation of 'p'

        // Compute BDD restrictions (XOR)
        bool contains_pos = (bdd_restrict(edge_cond, np) == bddfalse);
        bool contains_neg = (bdd_restrict(edge_cond, p) == bddfalse);

        if (!contains_pos && !contains_neg) {
            continue; // Neither condition affects the result
        }
        // Get the atomic proposition name
        string ap_name = domain_manager_->get_ap_name(i);

        // Bind the predicate
        auto predicate = extract_predicate_from_ap_name(ap_name);

        if (predicate == nullptr) {
            cerr << "Predicate not found for AP: " << ap_name << endl;
            continue;
        }

        // Set boolean value based on the presence of positive or negative conditions
        bound_predicates[predicate] = contains_pos;
    }
    return bound_predicates;
}

map<pddlboat::PredicatePtr, bool> PDDLProblem::collect_bound_predicates_in_OR(bdd& edge_cond) {
    // Vector to collect bound predicates (as a map from PredicatePtrs to a boolean)
    map<pddlboat::PredicatePtr, bool> bound_predicates;

    // Iterate over each individual disjunction element
    for (size_t i = 0; i < bdd_dict_->var_map.size() + 1; ++i) {
        bdd p = bdd_ithvar(i);  // Represents the atomic proposition 'p'
        bdd np = bdd_nithvar(i); // Represents the negation of 'p'

        // Check if this variable is part of the disjunction
        if (bdd_simplify(edge_cond, p) == edge_cond) {
            continue;  // Skip variables that are not part of the disjunction
        }

        // Get the atomic proposition name
        string ap_name = domain_manager_->get_ap_name(i);

        // Extract the predicate from the atomic proposition name
        auto predicate = extract_predicate_from_ap_name(ap_name);
        if (predicate == nullptr) {
            cerr << "Predicate not found for AP: " << ap_name << endl;
            continue;
        }

        // Add the predicate to the list of bound predicates
        bool is_positive = (bdd_restrict(bdd_not(edge_cond), p) == bddfalse);
        bound_predicates[predicate] = is_positive;
    }

    return bound_predicates;
}

void PDDLProblem::get_goals(const map<pddlboat::PredicatePtr, bool>& bound_predicates, vector<pddlboat::ExpressionPtr>& goals) {
    for (const auto& pred_pair : bound_predicates) {
        // Get the predicate and its associated boolean value 
        // (true for positive, false for negated)
        auto predicate = pred_pair.first;
        bool is_positive = pred_pair.second;

        // Create a literal according to its sign.
        pddlboat::ExpressionPtr signed_predicate = predicate;
        if (!is_positive) {
            signed_predicate = pddlboat::makeNot(predicate);
        }

        // Add the predicate to the goals 
        // (since it's disjunction, no constraints here)
        goals.push_back(signed_predicate);
    }
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
            // std::cout << "Predicate is satisfied in the start_state: ";
            // signed_predicate->toPDDL(std::cout) << std::endl;
            // The start state satisfies this predicate, so it's a constraint.
            constraints.push_back(signed_predicate);
        } else {
            // std::cout << "Predicate is NOT satisfied in the start_state: ";
            // signed_predicate->toPDDL(std::cout) << std::endl;
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

    // cout << "Added derived predicate: " << constraint_predicate_name << endl;

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

void PDDLProblem::write_solution_to_file(const string& file_path) const {
    // Open the file
    std::ofstream ofs(file_path);

    // Check if the file is open
    if (!ofs.is_open()) {
        std::cerr << "ERROR: failed to open file: " << file_path << std::endl;
        return;
    }
    // Write the plan = a sequence of actions.
    ofs << "Plan:" << std::endl;
    for (const auto& ps : product_path_) {
        if (auto next_step = ps.get_step()) {
            next_step->toString(ofs) << std::endl;
        }
    }
    ofs << std::endl;
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

    cout << "Solution written to " << file_path << endl;
}

string PDDLProblem::extract_problem_name(const string& path, const string& extension) {
    size_t lastSlashPos = path.find_last_of('/');
    size_t extensionPos = path.find(extension);

    if (lastSlashPos != std::string::npos && extensionPos != std::string::npos) {
        return path.substr(lastSlashPos + 1, extensionPos - lastSlashPos - 1);
    } else {
        std::cerr << "Invalid file path format: " << path << std::endl;
        return "problem"; // Default name in case of error
    }
}

string PDDLProblem::extract_domain_name(const string& path) {
    size_t lastSlashPos = path.find_last_of('/');
    // Extracting the domain name
    if (lastSlashPos != std::string::npos) {
        size_t secondLastSlashPos = path.find_last_of('/', lastSlashPos - 1);
        if (secondLastSlashPos != std::string::npos) {
            return path.substr(secondLastSlashPos + 1, lastSlashPos - secondLastSlashPos - 1);
        }
    }
    std::cerr << "Invalid file path format for domain name: " << path << std::endl;
    return "domain"; // Default name
}


void PDDLProblem::extract_names(const std::string& problemFile) {
    problem_name_ = extract_problem_name(problemFile);
    domain_name_ = extract_domain_name(problemFile);
}

// Helper function to validate the DFA trace
bool PDDLProblem::validate_dfa_trace(const vector<size_t>& dfa_trace) {
    if (dfa_trace.empty()) {
        cerr << "ERROR: the DFA trace is empty!" << endl;
        return false;
    } else if (dfa_trace.size() == 1) {
        cerr << "ERROR: the DFA trace is of size 1!" << endl;
        return false;
    }
    return true;
}

// Helper function to generate heuristics based on landmarks
int PDDLProblem::compute_heuristics(const ProductState& state, const map<size_t, DomainStateSet>& landmarks, size_t dfa_state) {
    if (landmarks.count(dfa_state)) {
        return state.compute_heuristic_cost(landmarks.at(dfa_state));
    }
    return 0;
}

// Helper function to handle wrong transitions
bool PDDLProblem::handle_wrong_transitions(size_t& curr_num_wrong_dfa_trans,
                                           shared_ptr<DFANode>& end_dfa_node, shared_ptr<DFANode>& next_dfa_node) {
    curr_num_wrong_dfa_trans++;
    if (curr_num_wrong_dfa_trans > max_num_wrong_dfa_trans_) {
        dfa_manager_->give_up_on_path(end_dfa_node, next_dfa_node);
        // cout << "Giving up on transition: " << next_dfa_node->getParent()->getState() << "=>" << next_dfa_node->getState() << endl;
        return false;  // Giving up on this trace.
    }
    return true;
}

// Helper function to push a new product state onto the corresponding queue
void PDDLProblem::enqueue_product_state(map<size_t, priority_queue<pair<int, ProductState>, vector<pair<int, ProductState>>, greater<pair<int, ProductState>>>>& regionQueues, const ProductState& state, int heuristic_cost, size_t dfa_state) {
    regionQueues[dfa_state].push(make_pair(heuristic_cost, state));
}

shared_ptr<pddlboat::Planner> PDDLProblem::get_task_planner(const pddlboat::ProblemPtr& subproblem) const {
    shared_ptr<pddlboat::Planner> task_planner;

    if (planner_type_ == "z3") {
        pddlboat::Z3Planner::Options options;
        options.dump_clauses = false;
        task_planner = make_shared<pddlboat::Z3Planner>(subproblem, options);

    } else if (planner_type_ == "fd") {
        pddlboat::FastDownwardPlanner::Options options;
        // Configure search type for Fast Downward planner
        if (search_type_ == "lazygreedy") {
            options.search = pddlboat::FastDownwardPlanner::LAZYGREEDY;
        } else if (search_type_ == "fdat") {
            options.search = pddlboat::FastDownwardPlanner::FDAT;
        } else if (search_type_ == "lama") {
            options.search = pddlboat::FastDownwardPlanner::LAMA;
        } else {
            cerr << "Unknown search type: " << search_type_ << endl;
            exit(EXIT_FAILURE);
        }

        task_planner = make_shared<pddlboat::FastDownwardPlanner>(subproblem, options);

    } else if (planner_type_ == "ff") {
        // Fast Forward planner
        task_planner = make_shared<pddlboat::FastForwardPlanner>(subproblem);

    } else if (planner_type_ == "symk") {
        // SymK planner
        task_planner = make_shared<pddlboat::SymKPlanner>(subproblem);

    } else if (planner_type_ == "astar") {
        // A* planner
        task_planner = make_shared<pddlboat::AStarPlanner>(subproblem);

    } else {
        cerr << "Unknown planner type: " << planner_type_ << endl;
        exit(EXIT_FAILURE);
    }

    return task_planner;
}

