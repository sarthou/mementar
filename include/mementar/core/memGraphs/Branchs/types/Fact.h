#ifndef MEMENTAR_FACT_H
#define MEMENTAR_FACT_H

#include <ostream>
#include <regex>
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

    Fact(const Fact& other) = default;

    Fact& operator=(const Fact& other)
    {
      this->subject_ = other.subject_;
      this->predicat_ = other.predicat_;
      this->object_ = other.object_;
      this->add_ = other.add_;

      this->t_ = other.t_;
      this->t_start_ = other.t_start_;
      this->t_end_ = other.t_end_;

      return *this;
    }

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
      if(std::regex_match(str, fact_match, fact_regex))
      {
        if(fact_match[3].str().empty())
          return Fact(Triplet::deserialize(fact_match[4].str()), SoftPoint(fact_match[1].str()));
        else
          return Fact(Triplet::deserialize(fact_match[4].str()), SoftPoint(fact_match[1].str(), fact_match[3].str()));
      }
      else
        return Fact("", 0);
    }

    static Fact* deserializePtr(const std::string& str)
    {
      if(std::regex_match(str, fact_match, fact_regex))
      {
        if(fact_match[3].str().empty())
          return new Fact(Triplet::deserialize(fact_match[4].str()), SoftPoint(fact_match[1].str()));
        else
          return new Fact(Triplet::deserialize(fact_match[4].str()), SoftPoint(fact_match[1].str(), fact_match[3].str()));
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
    static std::regex fact_regex;
    static std::smatch fact_match;
  };

} // namespace mementar

#endif // MEMENTAR_FACT_H
