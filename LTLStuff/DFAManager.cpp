#include "DFAManager.h"
#include <fstream>
#include <iostream>
#include <graphviz/gvc.h>

using namespace std;

DFAManager::DFAManager(shared_ptr<spot::bdd_dict> bddDict, set<bdd, BddComparator> equivalence_regions, bool feedback, bool hamming_dist)
    : bdd_dict_(bddDict), equivalence_regions_(equivalence_regions),
    feedback_(feedback), hamming_dist_(hamming_dist), use_pred_mapping_(false) {}

DFAManager::DFAManager(shared_ptr<spot::bdd_dict> bddDict, bool feedback, bool hamming_dist)
    : bdd_dict_(bddDict), feedback_(feedback), hamming_dist_(hamming_dist),use_pred_mapping_(true) {}

void DFAManager::construct_dfa(const LTLFormula& formula) {
    spot::formula spot_formula = formula.get_spot_formula();
    if (spot_formula == nullptr) {
        cerr << "Failed to parse the LTL formula" << endl;
        return;
    }
    // Create a translator instance. 
    spot::translator trans(bdd_dict_);
    // Set a type to BA (Büchi Automaton).
    trans.set_type(spot::postprocessor::Buchi);
    // Small (default) and Deterministic are exclusive choices for set_pref().
    // Indicate whether a smaller non-deterministic automaton should be
    // preferred over a deterministic automaton (no guarantees tho).
    trans.set_pref(spot::postprocessor::Deterministic); 
    // The automaton must use a state-based acceptance.
    trans.set_pref(spot::postprocessor::SBAcc);
    // Translate LTL formula to a DFA using Spot tranlsator.
    spot::twa_graph_ptr translated_dfa = trans.run(spot::from_ltlf(spot_formula));

    dfa_ = spot::to_finite(translated_dfa);
}

void DFAManager::save_dfa(const std::string& filename) {

    string dot_filename = filename + ".dot";
    string png_filename = filename + ".png";

    ofstream out(dot_filename);
    spot::print_dot(out, dfa_);
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

void DFAManager::print_dfa() {
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

size_t DFAManager::get_num_states() const {
    return dfa_->num_states();
}

size_t DFAManager::get_start_state() const {
    return dfa_->get_init_state_number();
}

bool DFAManager::is_accepting_state(size_t state) const {
    return dfa_->state_is_accepting(state);
}

spot::internal::state_out<spot::digraph<spot::twa_graph_state, spot::twa_graph_edge_data>> DFAManager::get_transitions(size_t state) const {
    return dfa_->out(state);
}

bdd DFAManager::get_self_edge_cond(size_t dfa_state) const{
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

shared_ptr<DFANode> DFAManager::generate_dfa_path() {

    while (!nodePriorityQueue_.empty()) {
        auto currentNodePair = nodePriorityQueue_.top();
        nodePriorityQueue_.pop();
        auto currentNode = currentNodePair.second;

        // Delete the handle for this node in the map.
        node_handles_.erase(currentNode);

        size_t current_dfa_state = currentNode->getState();

        if (is_accepting_state(current_dfa_state)) {
            auto dfa_trace = currentNode->getPathFromRoot();
            cout << "Selected a DFA trace of size " << dfa_trace.size() - 1 << " and a cost of " << currentNodePair.first << endl;
            print_dfa_path(dfa_trace);
            return currentNode;
        }
        for (auto& edge: get_transitions(current_dfa_state)) {
            size_t next_dfa_state = edge.dst;
            // TODO: PDDL doesn't support is_transition_feasible() yet.
            if (next_dfa_state != current_dfa_state && is_transition_feasible(edge.cond)) {
                int edge_cost;
                // Calculate the cost for this transition
                if (hamming_dist_) {
                    edge_cost = dfa_transition_cost(current_dfa_state, next_dfa_state, currentNode->getSelfEdgeCondition(), edge.cond);
                } else {
                    edge_cost = dfa_transition_cost(current_dfa_state, next_dfa_state);
                }
            
                int total_cost = currentNodePair.first + edge_cost;

                auto childNode = make_shared<DFANode>(next_dfa_state, currentNode, edge_cost, edge.cond);
                currentNode->addChild(childNode);

                auto handle = nodePriorityQueue_.push(make_pair(total_cost, childNode));
                node_handles_[childNode] = handle;
            }
        }
    }
    return nullptr; // Return empty if no path is found
}

// Define the transition_cost function based on past experiences.
int DFAManager::dfa_transition_cost(size_t from_state, size_t to_state) {
    if (!feedback_) return DEFAULT_COST;
    auto it = dfa_transition_costs_.find(make_pair(from_state, to_state));
    if (it != dfa_transition_costs_.end()) {
        // Return the stored cost if available
        return it->second;
    } else {
        // TODO: implement a cost based on the number of differing props.
        // Default cost if the transition has not been attempted yet
        return DEFAULT_COST; // = 1
    }
}

// Define the transition_cost function based on past experiences.
int DFAManager::dfa_transition_cost(size_t from_state, size_t to_state, const bdd& curr_state, const bdd& trans_edge_cond) {
    if (!feedback_) return DEFAULT_COST;
    auto it = dfa_transition_costs_.find(make_pair(from_state, to_state));
    if (it != dfa_transition_costs_.end()) {
        // Return the stored cost if available
        return it->second;
    } else {
        // Calculate the XOR of curr_state and trans_edge_cond
        bdd difference = curr_state ^ trans_edge_cond;

        // Count the number of differing propositions - this method will depend on your BDD library
        // int num_differing_props = count_true_propositions(difference); // = 1
        return DEFAULT_COST; 
    }
}

void DFAManager::update_dfa_transition_cost(shared_ptr<DFANode>& node, int cost) {

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

void DFAManager::update_dfa_transition_cost(size_t dfaStateOut, size_t dfaStateIn, int cost) {

    if (!feedback_) {
        // We don't want to provide feedback when this flag is set to false.
        return;
    }

    // Update the cost in the cost table.
    dfa_transition_costs_[make_pair(dfaStateOut, dfaStateIn)] = cost;
}

// Initialize all paths from the root = the start dfa state.
void DFAManager::initialize_node_priority_queue() {
    auto root = make_shared<DFANode>(get_start_state());
    auto handle = nodePriorityQueue_.push(make_pair(0, root));
    node_handles_[root] = handle;
}

void DFAManager::print_dfa_path(vector<size_t> dfa_path) const {
    std::cout << "DFA Path:" << endl;
    if (!dfa_path.empty()) {
        std::cout << dfa_path.front();
        for (auto it = next(dfa_path.begin()); it != dfa_path.end(); ++it) {
            std::cout << " -> " << *it;
        }
    }
    std::cout << endl;
}

void DFAManager::print_node_priority_queue() { 
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

bool DFAManager::is_transition_valid(const bdd& edge_cond, const bdd& next_state_bdd) {
    // next_state_bdd is a full assignment
    // edge_cond is a partial assignment
    // check if "next_state_bdd => edge_cond"
    bdd implication = bdd_apply(bdd_not(next_state_bdd), edge_cond, bddop_or);
    return implication == bddtrue;
}

bool DFAManager::is_transition_feasible(const bdd& edge_cond) {
    if (use_pred_mapping_) {
        // Equivalence regions are not used.
        // TODO: come up with something else.
        return true;
    }
    for (const auto& region_bdd : equivalence_regions_) {
        if (is_transition_valid(edge_cond, region_bdd)) {
            // At least one equivalence region satisfies the transition.
            return true;
        }
    }
    return false;
}

spot::twa_graph::edge_storage_t* DFAManager::find_transition(const bdd& next_state_bdd, size_t curr_dfa_state) {

    // Iterate over outgoing transitions of the current DFA state
    for (auto& edge : get_transitions(curr_dfa_state)) {
        if (is_transition_valid(edge.cond, next_state_bdd)) {
            // Found a valid transition
            return &edge;
        }
    }
    // If no valid transition is found
    return nullptr;
}