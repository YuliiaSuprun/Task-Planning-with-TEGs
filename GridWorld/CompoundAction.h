#ifndef COMPOUND_ACTION_H
#define COMPOUND_ACTION_H

#include<vector>
#include<set>
#include <iostream>
#include <bddx.h>

#include "Action.h"
#include "ProductState.h"

using namespace std;

/*
    We can store the projected "abstract" roadmap here along with the key.
*/
class CompoundAction : public Action {
public:
    CompoundAction(vector<ProductState> path) : path_(path) {
        std::cout << "CompoundAction: initialized" << endl;
        print_path();
        for (auto& state : path_) {
            state.cache();
        }
    }

    vector<ProductState> cached_path(bool with_end = false) const {

        if (with_end || path_.empty()) {
            return path_;
        } else {
            // Returning the path excluding the first state
            return vector<ProductState>(path_.begin(), path_.end() - 1);
        }
    }

    bool operator==(const CompoundAction& other) const {
        return path_ == other.path_;
    }

    bool operator<(const CompoundAction& other) const {
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
};

#endif // COMPOUND_ACTION_H
