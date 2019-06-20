#ifndef MEMENTAR_COMPRESSEDLEAFSESSION_H
#define MEMENTAR_COMPRESSEDLEAFSESSION_H

#include "mementar/core/archiving_compressing/archiving/Archive.h"
#include "mementar/core/archiving_compressing/archiving/Header.h"

#include "mementar/core/Btree/Btree.h"
#include "mementar/core/Fact.h"

namespace mementar
{

class CompressedLeafSession
{
public:
  CompressedLeafSession(const time_t& key, size_t index);

  time_t getKey() { return key_; }
  size_t getIndex() { return index_; }

  Btree<time_t, Fact>* getTree(Header& header, Archive& arch);
  std::vector<char> getRawData(Header& header, Archive& arch);
private:
  time_t key_;
  size_t index_;
};

} // namespace mementar

#endif // MEMENTAR_COMPRESSEDLEAFSESSION_H