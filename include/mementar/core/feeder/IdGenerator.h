#ifndef MEMENTAR_IDGENERATOR_H
#define MEMENTAR_IDGENERATOR_H

#include <sstream>
#include <string>

namespace mementar {

  class IdGenerator
  {
  public:
    IdGenerator() : id_(0) {}

    void inspect(const std::string& str_id)
    {
      std::stringstream convertor;
      size_t id = 0;
      convertor << str_id;
      convertor >> id;

      if(convertor.fail() == false)
      {
        if(id >= id_)
          id_ = id + 1;
      }
    }

    void reset() { id_ = 0; }

    std::string get()
    {
      return std::to_string(id_++);
    }

  private:
    size_t id_;
  };

} // namespace mementar

#endif // MEMENTAR_IDGENERATOR_H
