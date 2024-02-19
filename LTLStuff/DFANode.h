// #ifndef DFANODE_H
// #define DFANODE_H

// #include <cstddef>
// #include <memory>
// #include <spot/twa/bddprint.hh>
// #include <bddx.h>

// class DFANode {
// public:
//     // Constructor
//     DFANode(size_t state, std::shared_ptr<DFANode> parent, const bdd& edgeCond) : state_(state), parent_(parent), edgeCond_(edgeCond) {}

//     // Getter functions
//     size_t getState() const {
//         return state_;
//     }
//     std::shared_ptr<DFANode> getParent() const {
//         return parent_;
//     }
//     const bdd& getEdgeCondition() const;
//     const bdd& getSelfEdgeCondition() const;

//     // Setter funtion.
//     const bdd& setSelfEdgeCondition() const;

// private:
//     size_t state_;
//     std::shared_ptr<DFANode> parent_;
//     bdd edgeCond_;
//     bdd selfEdgeCond_;
// };

// #endif // DFANODE_H
