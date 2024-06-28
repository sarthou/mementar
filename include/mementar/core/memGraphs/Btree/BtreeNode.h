#ifndef MEMENTAR_BTREENODE_H
#define MEMENTAR_BTREENODE_H

#include <vector>

namespace mementar {

  template<typename Tkey, typename Tleaf>
  class BtreeLeafBase;

  template<typename Tkey, typename Tleaf>
  class BtreeNode
  {
  public:
    BtreeNode() : mother_(nullptr) {}
    ~BtreeNode()
    {
      for(auto* child : childs_)
        delete child;

      for(auto* leaf : leafs_)
        delete leaf;
    }

    bool isLeafNode() { return leafs_.empty() == false; }

    void setMother(BtreeNode<Tkey, Tleaf>* mother) { mother_ = mother; }
    BtreeNode<Tkey, Tleaf>* getMother() { return mother_; }

    std::vector<Tkey> keys_;
    BtreeNode<Tkey, Tleaf>* mother_;
    std::vector<BtreeNode<Tkey, Tleaf>*> childs_;
    std::vector<BtreeLeafBase<Tkey, Tleaf>*> leafs_;
  };

} // namespace mementar

#endif // MEMENTAR_BTREENODE_H
