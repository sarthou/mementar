#ifndef MEMENTAR_FACT_H
#define MEMENTAR_FACT_H

#include <ostream>
#include <string>

#include "mementar/core/memGraphs/Branchs/types/SoftPoint.h"
#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

namespace mementar {

  class Fact : public SoftPoint,
               public Triplet
  {
  public:
    Fact(const std::string& data, SoftPoint::Ttime t_start, std::optional<SoftPoint::Ttime> t_end = std::nullopt) : SoftPoint(t_start, t_end), Triplet(data)
    {}

    Fact(const std::string& data, const SoftPoint& soft_point) : SoftPoint(soft_point), Triplet(data)
    {}

    Fact(const Triplet& triplet, const SoftPoint& soft_point) : SoftPoint(soft_point), Triplet(triplet)
    {}

    static std::string serialize(const Fact& fact)
    {
      if(fact.isInstantaneous())
        return "[" + std::to_string(fact.getTimeStart()) + "]{" + Triplet::serialize(&fact) + "}";
      else
        return "[" + std::to_string(fact.getTimeStart()) + "," + std::to_string(fact.getTimeEnd()) + "]{" + Triplet::serialize(&fact) + "}";
    }

    static std::string serialize(const Fact* fact)
    {
      if(fact->isInstantaneous())
        return "[" + std::to_string(fact->getTimeStart()) + "]{" + Triplet::serialize(fact) + "}";
      else
        return "[" + std::to_string(fact->getTimeStart()) + "," + std::to_string(fact->getTimeEnd()) + "]{" + Triplet::serialize(fact) + "}";
    }

    static Fact deserialize(const std::string& str)
    {
      if(std::regex_match(str, match, regex))
      {
        if(match[3].str().empty())
          return Fact(Triplet::deserialize(match[4].str()), SoftPoint(std::stoul(match[1].str())));
        else
          return Fact(Triplet::deserialize(match[4].str()), SoftPoint(std::stoul(match[1].str()), std::stoul(match[3].str())));
      }
      else
        return Fact("", 0);
    }

    static Fact* deserializePtr(const std::string& str)
    {
      if(std::regex_match(str, match, regex))
      {
        if(match[3].str().empty())
          return new Fact(Triplet::deserialize(match[4].str()), SoftPoint(std::stoul(match[1].str())));
        else
          return new Fact(Triplet::deserialize(match[4].str()), SoftPoint(std::stoul(match[1].str()), std::stoul(match[3].str())));
      }
      else
        return nullptr;
    }

    std::string getData() const { return Triplet::toString(); }

    friend std::ostream& operator<<(std::ostream& os, Fact* fact)
    {
      os << fact->getData();
      return os;
    }

    bool operator==(const Fact& other) const
    {
      return Triplet::operator==(other);
    }

    bool operator==(const Fact* other) const
    {
      return Triplet::operator==(other);
    }

  protected:
    static std::regex regex;
    static std::smatch match;
  };

} // namespace mementar

#endif // MEMENTAR_FACT_H
