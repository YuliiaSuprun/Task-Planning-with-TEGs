#ifndef PDDL_PROBLEM_H
#define PDDL_PROBLEM_H

#include "PDDLDomain.h" 
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

#include <pddlboat/solver/planner.hpp>
#include <pddlboat/solver/fdplanner.hpp>
#include <pddlboat/solver/ffplanner.hpp>
#include <pddlboat/solver/symkplanner.hpp>
#include <pddlboat/solver/z3planner.hpp>
#include <pddlboat/solver/astarplanner.hpp>

using namespace std;

class PDDLProblem {
public:
    PDDLProblem(const string& problemFile, shared_ptr<PDDLDomain> domainPtr, bool cache=false, bool feedback=false, bool use_landmarks=false, bool hamming_dist=false, bool use_planner=false, bool save_dfa=false);
    ~PDDLProblem();

    // Method to get the wrapped pddlboat::Problem
    const pddlboat::ProblemPtr& getPddlboatProblemPtr() const;
    vector<ProductState> solve();
    vector<shared_ptr<DomainState>> get_domain_path() const;
    vector<size_t> get_dfa_path() const;

    string get_filename() const;
    double get_dfa_construction_time() const;
    size_t get_num_expanded_nodes() const;
    size_t get_num_generated_nodes() const;
    map<string, pair<string, vector<string>>> get_pred_mapping() const;

    void print_product_path() const;
    void print_domain_path() const;
    void print_dfa_path() const;

    string write_solution_to_file() const;

private:
    void extract_names(const string& problemFile);
    void parseProblem(const string& problemFile, shared_ptr<PDDLDomain> domainPtr);
    void save_paths();
    void realize_dfa_trace(shared_ptr<DFANode>& endTraceNode);
    void realize_dfa_trace_manually(shared_ptr<DFANode>& endTraceNode);
    void realize_dfa_trace_with_planner(shared_ptr<DFANode>& endTraceNode);
    pddlboat::ProblemPtr create_subproblem(bdd& edge_cond, shared_ptr<PDDLState> start_state);
    void print_bdd(bdd& expr);

    vector<ProductState> construct_path(const map<ProductState, vector<ProductState>>& parent_map, ProductState target_state, bool cached=false, size_t start_dfa_state=0);

    pddlboat::ProblemPtr pddlProblem_;
    LTLFormula formula_;
    shared_ptr<Domain> domain_;
    shared_ptr<PDDLState> start_domain_state_;
    bool cache_;
    bool feedback_;
    bool use_landmarks_;
    bool hamming_dist_;
    bool use_planner_;
    bool save_dfa_;
    
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
    string problem_name_;
    string domain_name_;
    chrono::duration<double> dfa_construction_time_;
    size_t num_expanded_nodes_;
};

#endif // PDDL_PROBLEM_H
