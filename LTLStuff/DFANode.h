#ifndef DFANODE_H
#define DFANODE_H

#include <vector>
#include <queue>
#include <map>
#include <memory>
#include <bddx.h>

#include <boost/heap/fibonacci_heap.hpp>

#include "Constants.h"

// Forward declare DFANode
class DFANode;

using NodeHeap = boost::heap::fibonacci_heap<
    std::pair<int, std::shared_ptr<DFANode>>,
    boost::heap::compare<std::greater<std::pair<int, std::shared_ptr<DFANode>>>>
>;


class DFANode : public std::enable_shared_from_this<DFANode>{
public:
    DFANode(size_t state, bdd selfEdgeCondition=bdd_false(), std::shared_ptr<DFANode> parent=nullptr, bdd parentEdgeCondition=bdd_true(), int parent_edge_cost=DEFAULT_COST);

    // Getters
    size_t getState() const;
    std::shared_ptr<DFANode> getParent() const;
    const std::vector<std::shared_ptr<DFANode>>& getChildren() const;

    int getParentEdgeCost() const;
    int getPathCost() const;

    bool updateParentEdgeCost(int new_parent_edge_cost, const std::map<std::shared_ptr<DFANode>, NodeHeap::handle_type>& node_handles, NodeHeap& node_priority_queue);
    
    bdd& getSelfEdgeCondition();
    bdd& getParentEdgeCondition();

    // Setters
    void addChild(std::shared_ptr<DFANode> child);
    void setSelfEdgeCondition(const bdd& condition);
    void setParentEdgeCondition(const bdd& condition);

    // Utility functions to get the DFA path
    std::vector<size_t> getPathFromRoot() const;
    std::vector<std::shared_ptr<DFANode>> getNodePathFromRoot();

    bool isReachable();

private:
    size_t dfa_state_;
    std::shared_ptr<DFANode> parent_;
    std::vector<std::shared_ptr<DFANode>> children_;

    int parent_edge_cost_;
    int path_from_root_cost_;

    bdd parent_edge_condition_;
    bdd self_edge_condition_;

    bool reachable_;
};

#endif // DFANODE_H
