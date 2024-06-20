#ifndef MEMENTAR_CSVSAVER_H
#define MEMENTAR_CSVSAVER_H

#include <string>

#include "mementar/core/memGraphs/Timeline.h"

namespace mementar {

  class CsvSaver
  {
  public:
    CsvSaver() = default;
    bool save(const std::string& file_name, Timeline* timeline);

  private:
    void replaceAll(std::string& text, const std::string& to_replace, const std::string& replace_with);
  };

} // namespace mementar

#endif // MEMENTAR_CSVSAVER_H