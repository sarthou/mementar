#include "mementar/core/memGraphs/Branchs/types/Fact.h"

#include <regex>

namespace mementar {

  std::regex Fact::fact_regex(R"(\[(\d*)(,(\d*))?\]\{([^\}]*)\})");
  std::smatch Fact::fact_match;

} // namespace mementar
