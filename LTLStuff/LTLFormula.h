#ifndef LTL_FORMULA_H
#define LTL_FORMULA_H

#include <string>
#include <map>
#include <set>
#include <iostream>

#include <spot/tl/formula.hh>
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>
#include "DomainState.h"

using namespace std;

/*
Documentation on using SPOT for LTLf formulas: 
https://spot.lre.epita.fr/tut12.html

General Approach:
1. Have Spot read the input LTLf formula as if it was LTL.
2. Rewrite this formula in a way that embeds the semantics of LTLf in LTL.
First, introduce a new atomic proposition alive that will be true initially,
but that will eventually become false forever. Then adjust all original LTL
operators so that they have to be satisfied during the alive part of the word.
For instance the formula (a U b) & Fc would be transformed into alive & (a U (alive & b)) & F(alive & c) & (alive U G!alive).
3. Convert the resulting formula into a Büchi automaton
4. Remove the alive property, after marking as accepting all states with
an outgoing edge labeled by !alive.
(Note that since Spot does not actually support state-based acceptance,
it needs to keep a false self-loop on any accepting state without a successor
in order to mark it as accepting.)
5. Interpret the above automaton as finite automaton.
*/


class LTLFormula {
public:
    LTLFormula(const string& formula, 
               const map<string, DomainStateSet>&ap_mapping={})
        : formula_(formula), ap_mapping_(ap_mapping) {}
    
    // Default constructor
    LTLFormula() : formula_(""), ap_mapping_() {}

    string get_formula() const { return formula_; }
    map<string, DomainStateSet> get_ap_mapping() const { return ap_mapping_; }

    // Function to return Spot's formula
    spot::formula get_spot_formula() const {
        spot::parsed_formula pf = spot::parse_infix_psl(formula_);
        // Here, we could also handle any parsing errors.
        (void) pf.format_errors(cerr);
        cout << "spot formula: " << pf.f << endl;
        return pf.f;
    }

    // Overloading the << operator.
    friend ostream& operator<<(ostream& os, const LTLFormula& ltlFormula);

    // Additional utility methods can be added as required

private:
    string formula_;
    map<string, DomainStateSet> ap_mapping_;
};

// Definition of the overloaded << operator for LTLFormula
inline ostream& operator<<(ostream& os, const LTLFormula& ltlFormula) {
    os << ltlFormula.formula_;
    return os;
}

#endif // LTL_FORMULA_H
