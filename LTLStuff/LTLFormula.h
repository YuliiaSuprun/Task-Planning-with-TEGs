#ifndef LTL_FORMULA_H
#define LTL_FORMULA_H

#include <string>
#include <map>
#include <set>
#include "GridState.h"

using namespace std;

class LTLFormula {
public:
    LTLFormula(const string& formula, 
               const map<string, set<GridState>>& ap_mapping)
        : formula_(formula), ap_mapping_(ap_mapping) {}

    string get_formula() const { return formula_; }
    map<string, set<GridState>> get_ap_mapping() const { return ap_mapping_; }

    // Overloading the << operator.
    friend ostream& operator<<(ostream& os, const LTLFormula& ltlFormula);

    // Additional utility methods can be added as required

private:
    string formula_;
    map<string, set<GridState>> ap_mapping_;
};

// Definition of the overloaded << operator for LTLFormula
ostream& operator<<(ostream& os, const LTLFormula& ltlFormula) {
    os << ltlFormula.formula_;
    return os;
}

#endif // LTL_FORMULA_H