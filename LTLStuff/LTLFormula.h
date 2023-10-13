#ifndef LTL_FORMULA_H
#define LTL_FORMULA_H

#include <string>
#include <map>
#include <set>
#include <iostream>

#include <spot/tl/formula.hh>
#include <spot/tl/parse.hh>
#include <spot/tl/print.hh>
#include "GridState.h"

using namespace std;

class LTLFormula {
public:
    LTLFormula(const string& formula, 
               const map<string, set<GridState>>& ap_mapping)
        : formula_(formula), ap_mapping_(ap_mapping) {}

    string get_formula() const { return formula_; }
    map<string, set<GridState>> get_ap_mapping() const { return ap_mapping_; }

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
    map<string, set<GridState>> ap_mapping_;
};

// Definition of the overloaded << operator for LTLFormula
inline ostream& operator<<(ostream& os, const LTLFormula& ltlFormula) {
    os << ltlFormula.formula_;
    return os;
}

#endif // LTL_FORMULA_H
