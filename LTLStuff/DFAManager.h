#ifndef DFAUTILITY_H
#define DFAUTILITY_H

#include <memory>
#include <string>

#include <spot/tl/formula.hh>
#include <spot/twaalgos/dot.hh>
#include <spot/twaalgos/translate.hh>
#include <spot/twaalgos/totgba.hh>
#include <spot/twaalgos/hoa.hh>
#include <spot/twaalgos/remprop.hh>
#include <spot/twaalgos/postproc.hh> 
#include <spot/twa/bddprint.hh>
#include <spot/tl/ltlf.hh>
#include <spot/twa/twagraph.hh>

#include <bddx.h>


// Forward declaration of required classes
class LTLFormula;

class DFAManager {
public:
    // Constructor
    DFAManager(std::shared_ptr<spot::bdd_dict> bddDict);

    // Convert LTL formula to DFA
    void construct_dfa(const LTLFormula& formula);

    // Save DFA to DOT and PNG files
    void save_dfa(const std::string& filename);

    void print_dfa();

    size_t get_num_states() const;
    size_t get_start_state() const;
    spot::internal::state_out<spot::digraph<spot::twa_graph_state, spot::twa_graph_edge_data>> get_transitions(size_t state) const;
    bool is_accepting(size_t state) const;

    bdd get_self_edge_cond(size_t dfa_state) const;

private:
    spot::bdd_dict_ptr bdd_dict_;

    // DFA corresponding to LTL formula.
    std::shared_ptr<spot::twa_graph> dfa_;
};

#endif // DFAUTILITY_H
