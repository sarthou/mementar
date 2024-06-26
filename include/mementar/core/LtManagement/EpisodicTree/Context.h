#ifndef MEMENTAR_CONTEXT_H
#define MEMENTAR_CONTEXT_H

#include <ctime>
#include <map>
#include <string>

#include "mementar/core/memGraphs/Branchs/types/Triplet.h"
#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"

namespace mementar {

  class Context
  {
  public:
    Context(SoftPoint::Ttime key) : key_(key) {}

    void insert(const Triplet* triplet);
    void remove(const Triplet* triplet);

    bool exist(const std::string& name);
    bool subjectExist(const std::string& subject);
    bool predicatExist(const std::string& predicat);
    bool objectExist(const std::string& object);

    std::string toString();
    void fromString(const std::string& string);

    SoftPoint::Ttime getKey() const { return key_; }
    void setKey(SoftPoint::Ttime key) { key_ = key; }

    static void storeContexts(std::vector<Context>& contexts, const std::string& directory);
    static std::string contextsToString(std::vector<Context>& contexts);
    static void loadContexts(std::vector<Context>& contexts, const std::string& directory);
    static std::vector<Context> stringToContext(const std::string& str);

  private:
    SoftPoint::Ttime key_;
    std::map<std::string, size_t> subjects_;
    std::map<std::string, size_t> predicats_;
    std::map<std::string, size_t> objects_;
  };

} // namespace mementar

#endif // MEMENTAR_CONTEXT_H
