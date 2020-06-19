#include "mementar/core/memGraphs/DoublyLinkedList/DllCargoNode.h"

#include "mementar/core/memGraphs/DoublyLinkedList/DllLinkedElement.h"

#include <iostream>

namespace mementar {

DllCargoNode::DllCargoNode() : DllNode()
{
}

void DllCargoNode::push_back(DllLinkedElement* data)
{
  data->dll_node_ = this;
  DllNode::push_back(data);
}

void DllCargoNode::remove(DllLinkedElement* data)
{
  for(size_t i = 0; i < payload_.size();)
  {
    if(payload_[i]->operator==(data))
    {
      unlinkDll(i);
      payload_.erase(payload_.begin() + i);
    }
    else
      i++;
  }
}

void DllCargoNode::unlinkDll(size_t i)
{
  payload_[i]->dll_node_ = nullptr;
}

} // namespace mementar
