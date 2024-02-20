#include "DFANode.h"

DFANode::DFANode(size_t state, std::shared_ptr<DFANode> parent, bdd selfEdgeCondition, bdd parentEdgeCondition)
    : dfa_state_(state), parent_(parent), self_edge_condition_(selfEdgeCondition), parent_edge_condition_(parentEdgeCondition) {}

size_t DFANode::getState() const {
    return dfa_state_;
}

std::shared_ptr<DFANode> DFANode::getParent() const {
    return parent_;
}

const std::vector<std::shared_ptr<DFANode>>& DFANode::getChildren() const {
    return children_;
}

bdd DFANode::getSelfEdgeCondition() const {
    return self_edge_condition_;
}

bdd DFANode::getParentEdgeCondition() const {
    return parent_edge_condition_;
}

void DFANode::addChild(std::shared_ptr<DFANode> child) {
    children_.push_back(child);
}

void DFANode::setSelfEdgeCondition(const bdd& condition) {
    self_edge_condition_ = condition;
}

void DFANode::setParentEdgeCondition(const bdd& condition) {
    parent_edge_condition_ = condition;
}

std::vector<size_t> DFANode::getPathToRoot() const {
    std::vector<size_t> path;
    for (auto node = this; node != nullptr; node = node->parent_.get()) {
        path.push_back(node->dfa_state_);
    }
    std::reverse(path.begin(), path.end());
    return path;
}
