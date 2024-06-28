#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

#include <regex>

namespace mementar {

  std::regex Triplet::regex(R"((\w)\|(\w+)\|(\w+)\|(\w+))");
  std::regex Triplet::regex2(R"(\[([^\]]+)\]([^|]+)\|([^|]+)\|([^|]+))");
  std::smatch Triplet::match;

} // namespace mementar
