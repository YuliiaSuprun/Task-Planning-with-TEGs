#include "DFANode.h"

DFANode::DFANode(size_t state, bdd selfEdgeCondition, std::shared_ptr<DFANode> parent, bdd parentEdgeCondition, int parent_edge_cost)
    : dfa_state_(state), parent_(parent),
    parent_edge_cost_(parent_edge_cost),
    parent_edge_condition_(parentEdgeCondition),
    self_edge_condition_(selfEdgeCondition),
    reachable_(true) {
        if (!parent_) {
            // This is a root node (initial dfa state).
            // The path cost is initialized to 0.
            parent_edge_cost_ = 0;
            path_from_root_cost_ = 0;
        } else {
            // This not the root node.
            // Compute the path effort score "recursively"
            path_from_root_cost_ = parent_edge_cost_ + parent->path_from_root_cost_;
        }
    }

size_t DFANode::getState() const {
    return dfa_state_;
}

std::shared_ptr<DFANode> DFANode::getParent() const {
    return parent_;
}

const std::vector<std::shared_ptr<DFANode>>& DFANode::getChildren() const {
    return children_;
}

int DFANode::getParentEdgeCost() const {
    return parent_edge_cost_;
}

int DFANode::getPathCost() const {
    return path_from_root_cost_;
}

bdd& DFANode::getSelfEdgeCondition() {
    return self_edge_condition_;
}

bdd& DFANode::getParentEdgeCondition() {
    return parent_edge_condition_;
}

bool DFANode::isReachable() {
    return reachable_;
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

std::vector<size_t> DFANode::getPathFromRoot() const {
    std::vector<size_t> path;
    for (auto node = this; node != nullptr; node = node->parent_.get()) {
        path.push_back(node->dfa_state_);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<std::shared_ptr<DFANode>> DFANode::getNodePathFromRoot() {
    std::vector<std::shared_ptr<DFANode>> nodePath;
    for (auto node = shared_from_this(); node != nullptr; node = node->parent_) {
        nodePath.push_back(node);
    }
    std::reverse(nodePath.begin(), nodePath.end());
    return nodePath;
}

bool DFANode::updateParentEdgeCost(int new_parent_edge_cost, const std::map<std::shared_ptr<DFANode>, NodeHeap::handle_type>& node_handles, NodeHeap& node_priority_queue) {

    if (new_parent_edge_cost == parent_edge_cost_) {
        // No changes are needed.
        return false;
    }

    // The scores are different: need to update all successors!
    int deltaScore = new_parent_edge_cost - parent_edge_cost_;
    parent_edge_cost_ = new_parent_edge_cost;

    if (parent_edge_cost_ == FAILURE_COST) {
        reachable_ = false;
    }

    std::queue<std::shared_ptr<DFANode>> queue;
    queue.push(shared_from_this());

    while (!queue.empty()) {
        auto currentNode = queue.front();
        queue.pop();

        // Update the path score of the current node
        currentNode->path_from_root_cost_ += deltaScore;
        // Propagate "reachability status" to successors.
        currentNode->reachable_ = reachable_;

        if (node_handles.count(currentNode) > 0) {
            // There is a path in the priority queue that ends on this node.
            auto handle = node_handles.at(currentNode);
            // Update the key value in the priority queue.
            (*handle).first = currentNode->path_from_root_cost_;
            // "Reorder" the elements in the priority queue to reflect this change. 
            node_priority_queue.update(handle);
        }

        // Enqueue all children of the current node
        for (const auto& child : currentNode->children_) {
            queue.push(child);
        }
    }

    return true;
}
