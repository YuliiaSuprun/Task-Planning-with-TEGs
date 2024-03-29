#ifndef TEG_TASK_H
#define TEG_TASK_H

#include "Domain.h"
#include "DomainState.h"
#include "ProductState.h"
#include "ProductTransition.h"
#include "LTLFormula.h"
#include "DFANode.h"
#include "DFAManager.h"
#include "DomainManager.h"
#include "ProductManager.h"
#include "DFAManager.h"
#include "Constants.h"

#include <map>
#include <vector>
#include <deque>
#include <queue>
#include <unordered_map>
#include <string>
#include <set>

using namespace std;

class TEGProblem {
public:
    TEGProblem(const string formula_str,
            const map<string, DomainStateSet> ap_to_states_mapping,
            const shared_ptr<Domain> domain,
            const shared_ptr<DomainState> start_domain_state,
            int problem_id = 0, bool on_the_fly=false, bool cache=false, bool feedback=false, bool use_landmarks=false);

    ~TEGProblem();
    
    vector<ProductState> solve();
    vector<shared_ptr<DomainState>> get_domain_path() const;
    vector<size_t> get_dfa_path() const;
    map<string, DomainStateSet> get_ap_to_states_mapping() const;
    string get_filename() const;

    void print_product_path() const;
    void print_domain_path() const;
    void print_dfa_path() const;

private:

    void solve_with_full_graph();
    void solve_with_on_the_fly_graph();
    void save_paths();

    void realize_dfa_trace(shared_ptr<DFANode>& endTraceNode);

    vector<ProductState> construct_path(const map<ProductState, vector<ProductState>>& parent_map, ProductState target_state, bool cached=false, size_t start_dfa_state=0);

    // Class members
    LTLFormula formula_;
    shared_ptr<Domain> domain_;
    shared_ptr<DomainState> start_domain_state_;
    int problem_id_;
    bool on_the_fly_;
    bool cache_;
    bool use_landmarks_;

    // DFA corresponding to LTL formula.
    spot::bdd_dict_ptr bdd_dict_;

    std::shared_ptr<DomainManager> domain_manager_;
    std::shared_ptr<DFAManager> dfa_manager_;
    std::shared_ptr<ProductManager> product_manager_;

    // Solution path (if found).
    vector<ProductState> product_path_;
    vector<shared_ptr<DomainState>> domain_path_;
    vector<size_t> dfa_path_;

    string filename_;
};

#endif // TEG_TASK_H
