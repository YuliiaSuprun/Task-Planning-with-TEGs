#ifndef DFANODE_H
#define DFANODE_H

#include <vector>
#include <memory>
#include <bddx.h>

class DFANode {
public:
    DFANode(size_t state, std::shared_ptr<DFANode> parent=nullptr, bdd selfEdgeCondition=bdd_true(), bdd parentEdgeCondition=bdd_true());

    // Getters
    size_t getState() const;
    std::shared_ptr<DFANode> getParent() const;
    const std::vector<std::shared_ptr<DFANode>>& getChildren() const;
    bdd getSelfEdgeCondition() const;
    bdd getParentEdgeCondition() const;

    // Setters
    void addChild(std::shared_ptr<DFANode> child);
    void setSelfEdgeCondition(const bdd& condition);
    void setParentEdgeCondition(const bdd& condition);

    // Utility function to get the DFA path
    std::vector<size_t> getPathToRoot() const;

private:
    size_t dfa_state_;
    std::shared_ptr<DFANode> parent_;
    std::vector<std::shared_ptr<DFANode>> children_;
    bdd self_edge_condition_;
    bdd parent_edge_condition_;
};

#endif // DFANODE_H
