#include "PDDLProblem.h"
#include <pddlboat/parser/parser.hpp>
#include <pddlboat/parser/translator.hpp>
#include <iostream>
#include <sstream>

PDDLProblem::PDDLProblem(const string& problemFile, shared_ptr<PDDLDomain> domainPtr, bool cache, bool feedback, bool use_landmarks, bool hamming_dist, bool use_planner, int problem_id) 
: domain_(domainPtr), problem_id_(problem_id), cache_(cache),
feedback_(feedback), use_landmarks_(use_landmarks),
hamming_dist_(hamming_dist), use_planner_(use_planner), bdd_dict_(make_shared<spot::bdd_dict>()), num_expanded_nodes_(0) {

    // Extracting the problem name
    size_t lastSlashPos = problemFile.find_last_of('/');
    size_t extensionPos = problemFile.find(".pddl");
    if (lastSlashPos != string::npos && extensionPos != string::npos) {
        problem_name_ = problemFile.substr(lastSlashPos + 1, extensionPos - lastSlashPos - 1);
    } else {
        cerr << "Invalid problem file path format." << endl;
        problem_name_ = ""; // Set to an empty string or handle the error as you see fit
    }

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

    cout << "Formula: " << formula_ << endl;

    // Compute the DFA corresponding to the given LTL formula.
    dfa_manager_->construct_dfa(formula_);

    auto end1 = chrono::high_resolution_clock::now();
    dfa_construction_time_ = end1 - start1;
    // std::cout << "time of calculating the automaton: " << dfa_construction_time_.count() << " seconds" << std::endl;

    // dfa_manager_->print_dfa();
    dfa_manager_->save_dfa(filename_);

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

    if (use_planner_) {
        // 1. Create an instanse of the subproblem 
        // 2. Call the off-the-shelf planner
        realize_dfa_trace_with_planner(endTraceNode);
    } else {
        realize_dfa_trace_manually(endTraceNode);
    }
}

void PDDLProblem::realize_dfa_trace_manually(shared_ptr<DFANode>& endTraceNode) {

    cout << "realize_dfa_trace_manually" << endl;

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
        if (queue.size() == 1) {
            // cout << "Trying to realize this transition: " << curr_dfa_state << "=>" << dfa_trace.at(currentRegionIndex+1) << endl;
        }
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

    // size_t max_num_wrong_dfa_trans = 300;

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
    // size_t curr_num_wrong_dfa_trans = 0;



    while (!regionSubproblems.empty()) {
        size_t curr_dfa_state = dfa_trace.at(currentRegionIndex);
        size_t next_dfa_state = dfa_trace.at(currentRegionIndex + 1);

        cout << "Trying to realize a transition: " << curr_dfa_state << "=>" << next_dfa_state << endl;
        if (regionSubproblems.count(curr_dfa_state) == 0) {
            // This problem can't be solved.
            cout << "Backtracking" << endl;
            // Backtrack if necessary
            currentRegionIndex--;
            continue;  
        }

        auto& subproblem = regionSubproblems[curr_dfa_state];

        // Solve the problem
        // pddlboat::Z3Planner::Options options;
        // options.dump_clauses = false;
        // options.horizon.max = 4;
        // auto task_planner = make_shared<pddlboat::Z3Planner>(subproblem, options);

        // auto task_planner = make_shared<pddlboat::FastDownwardPlanner>(subproblem);

        // auto task_planner = make_shared<pddlboat::FastForwardPlanner>(subproblem);

        // auto task_planner = make_shared<pddlboat::SymKPlanner>(subproblem);

        auto task_planner = make_shared<pddlboat::AStarPlanner>(subproblem);
 
        cout << "Plan:" << endl;
        auto plan = make_shared<pddlboat::Plan>(subproblem);
        if (!task_planner->solve(*plan))
        {
            cerr << "Failed to solve!" << endl;
            regionSubproblems.erase(curr_dfa_state);
            continue;
        } else {
            cout << *plan << endl;
        }

        // Get a trace of states from the plan.
        auto state_trace = plan->getStepStates(true);
        vector<shared_ptr<DomainState>> domain_path;
        vector<ProductState> product_path;

        if (state_trace.empty()) {
            cerr << "ERROR: the plan is empty!" << endl;
        }
        // Iterate over all domain states except the last one
        for (int i = state_trace.size() - 2; i >= 0; --i) {
            auto domain_state = make_shared<PDDLState>(state_trace[i]);
            domain_path.push_back(domain_state);
            product_path.emplace_back(domain_state, curr_dfa_state);
        }
        
        // Handle the last domain state separately
        auto last_domain_state = make_shared<PDDLState>(state_trace.back());
        ProductState next_state(last_domain_state, next_dfa_state);
        // TODO: verify domain_path and augment elems with dfa states.

        // Check if it was already explored.
        if (visited.find(next_state) != visited.end()) {
            // Skip this state.
            // TODO: modify the problem to avoid this domain state?
            continue;
        }

        // We were able to get to the next DFA state in a trace!
        cout << "Succesfully realized this transition: " << curr_dfa_state << "=>" << next_dfa_state << endl;

        // Add it to the parent map to be able to backtrack.
        parent_map.emplace(next_state, product_path);
        // cout << "emplaced state: " << next_state << endl;

        // Make all states in the path visited.
        for (const auto& product_state : product_path) {
            visited.insert(product_state);
        }

        // Update the cost to 0 based on the success of the transition.
        dfa_manager_->update_dfa_transition_cost(dfa_nodes.at(currentRegionIndex + 1), SUCCESS_COST);

        // TODO: Optimization 1: cache paths that cross the dfa states

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
            product_path_ = construct_path(parent_map, next_state);
            save_paths();
            return;
        }

        // TODO: instantiate a subproblem.
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

pddlboat::ProblemPtr PDDLProblem::create_subproblem(bdd& edge_cond, shared_ptr<PDDLState> start_state) {
    // cout << "Edge_condition: " << edge_cond << endl;

    pddlboat::ProblemPtr subproblem(pddlProblem_);
    // Vector to collect bound predicates
    vector<pddlboat::ExpressionPtr> bound_predicates;

    for (size_t i = 0; i < bdd_dict_->var_map.size(); ++i) {
        bdd p = bdd_ithvar(i);  // Represents the atomic proposition 'p'
        bdd np = bdd_nithvar(i); // Represents the negation of 'p'
        
        if ((bdd_restrict(edge_cond, np) != bddfalse) && (bdd_restrict(edge_cond, p) != bddfalse)) {
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

        // The first part is the name of the predicate
        string predicate_name = tokens[0];
        vector<string> bindings(tokens.begin() + 1, tokens.end());

        // Find the predicate definition
        auto predicate_it = pddlProblem_->domain->predicates.find(predicate_name);
        if (predicate_it == pddlProblem_->domain->predicates.end()) {
            std::cerr << "Predicate not found: " << predicate_name << std::endl;
            continue;
        }
        auto p_predicate = predicate_it->second;

        // Check if including the proposition (or its negation) makes a difference in the XOR result
        if (bdd_restrict(edge_cond, np) == bddfalse) {
            // Condition contains positive p
            std::cout << "Found a positive ap: " << ap_name << std::endl;

            // Bind the predicate
            auto bound_pred = p_predicate->bind(bindings, *subproblem);
            bound_predicates.push_back(bound_pred);

        } else if (bdd_restrict(edge_cond, p) == bddfalse) {
            // Condition contains negative p: an obstacle
            std::cout << "Found a negative ap: " << ap_name << std::endl;

            // Bind the predicate and then negate it
            auto bound_pred = p_predicate->bind(bindings, *subproblem);
            auto negated_pred = pddlboat::makeNot(bound_pred);
            bound_predicates.push_back(negated_pred);
        }
    }

    // Finally, given a list of propositions (positive and negative), you should create a new goal
    subproblem->goal = pddlboat::makeAnd(bound_predicates);

    // auto domain = pddlProblem_->domain;
    // cout << "Domain: " << endl;
    // domain->toPDDL(std::cout) << std::endl;
    // cout << "Initial goal is: " << endl;
    // start_subproblem->goal->toPDDL(std::cout) << std::endl;
    // cout << "Printing predicates..." << endl;
    // for (const auto& pred_pair : domain->predicates) {
    //     auto name = pred_pair.first;
    //     auto pred = pred_pair.second;
    //     cout << "Name: " << name << endl;
    //     pred->toPDDL(std::cout) << std::endl;
    // }
    // auto p_on = pddlProblem_->domain->predicates.at("on");
    // // Bind the predicates 
    // auto on_d_c = p_on->bind({"b1", "b2"}, *subproblem);
    // auto on_c_b = p_on->bind({"b2", "b3"}, *subproblem);
    // // Set the goal
    // subproblem->goal = pddlboat::makeAnd({on_d_c, on_c_b});

    cout << "Subgoal is: " << endl;
    subproblem->goal->toPDDL(std::cout) << std::endl;

    subproblem->start = start_state->getPddlboatStatePtr();

    return subproblem;
}