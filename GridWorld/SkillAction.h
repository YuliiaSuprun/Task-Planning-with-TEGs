#ifndef SKILL_ACTION_H
#define SKILL_ACTION_H

#include<vector>
#include<set>
#include <iostream>
#include <bddx.h>

#include "ProductState.h"
#include "Action.h"

using namespace std;

/*
    We can store the projected "abstract" roadmap here along with the key.
*/
class SkillAction : public Action {
public:
    SkillAction(vector<ProductState> path, bdd self_edge_cond, bdd out_edge_cond) : path_(path), self_edge_cond_(self_edge_cond), out_edge_cond_(out_edge_cond) {
        std::cout << "SkillAction: initialized" << endl;
        print_path();
    }

    bool isSkillAction() const override { return true; }

    // Override the virtual function from Action
    vector<ProductState> projected_path(bool with_end = false) const {
        if (with_end || path_.empty()) {
            return path_;
        } else {
            // Returning the path excluding the first state
            return vector<ProductState>(path_.begin(), path_.end() - 1);
        }
    }

    vector<ProductState> reconstructed_path(size_t dfa_start_state, size_t dfa_end_state, bool with_end = false) const {
        if (path_.empty()) {
            return path_;
        } else {
            vector<ProductState> reconstructed_path;
            for (auto it = next(path_.begin()); it != (path_.end() - 1); ++it) {
                reconstructed_path.emplace_back(it->get_grid_state(), dfa_start_state);
            }

            if (with_end) {
                // Add the new end dfa state.
                reconstructed_path.emplace_back(path_.back().get_grid_state(), dfa_end_state);
            }    
            return reconstructed_path;
        }
    }

    bdd self_edge_cond() const { return self_edge_cond_; }
    bdd out_edge_cond() const { return out_edge_cond_; }

    GridState start_grid_state() const {
        return path_.front().get_grid_state();
    }

    GridState dest_grid_state() const {
        return path_.back().get_grid_state();
    }

    bool relaxedMatch(bdd new_self_edge_cond, bdd new_out_edge_cond) const {
        // Check if edges contradict each other.
        if (((self_edge_cond_ & new_self_edge_cond) == bddfalse) || ((out_edge_cond_ & new_out_edge_cond) == bddfalse)) {
            return false;
        }
        // Can additionally check "non-recoverable" states.
        return (out_edge_cond_ & new_self_edge_cond) == bddfalse;
    }

    bool constraintMatch(bdd new_self_edge_cond, bdd new_out_edge_cond) const {
        // Check if edges contradict each other.
        if (((self_edge_cond_ & new_self_edge_cond) == bddfalse) || ((out_edge_cond_ & new_out_edge_cond) == bddfalse)) {
            return false;
        }
        // Check if abstract edge conditions satisfy new edge conditions.
        return (self_edge_cond_ & new_self_edge_cond) == self_edge_cond_ && (out_edge_cond_ & new_out_edge_cond) == out_edge_cond_;
    }

    bool operator==(const SkillAction& other) const {
        return path_ == other.path_ && self_edge_cond_ == other.self_edge_cond_ && out_edge_cond_ == other.out_edge_cond_;
    }

    bool operator<(const SkillAction& other) const {
        // First compare by path size
        if (path_.size() != other.path_.size()) {
            return path_.size() < other.path_.size();
        }
        // Then lexicographically compare the paths
        for (size_t i = 0; i < path_.size(); ++i) {
            if (!(path_[i] == other.path_[i])) {
                return path_[i] < other.path_[i];
            }
        }
        // Compare the BDDs based on the BDD id, since BDDs don't have a natural ordering
        if (self_edge_cond_.id() != other.self_edge_cond_.id()) {
            return self_edge_cond_.id() < other.self_edge_cond_.id();
        }
        if (out_edge_cond_.id() != other.out_edge_cond_.id()) {
            return out_edge_cond_.id() < other.out_edge_cond_.id();
        }
        // If all above are equal, consider them equal
        return false;
    }


    void print_path() const {
        std::cout << "Cached Path:" << endl;
        if (!path_.empty()) {
            std::cout << path_.front();
            for (auto it = next(path_.begin()); it != path_.end(); ++it) {
                std::cout << " -> " << *it;
            }
        }
        std::cout << endl;
    }

private:
    vector<ProductState> path_;
    bdd self_edge_cond_;
    bdd out_edge_cond_;
};

#endif // SKILL_ACTION_H
