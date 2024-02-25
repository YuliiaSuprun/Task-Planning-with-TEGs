#include "DFAManager.h"
#include <fstream>
#include <iostream>
#include <graphviz/gvc.h>
#include "LTLFormula.h" 

using namespace std;

DFAManager::DFAManager(shared_ptr<spot::bdd_dict> bddDict)
    : bdd_dict_(bddDict) {}

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

bool DFAManager::is_accepting(size_t state) const {
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