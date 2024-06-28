#ifndef MEMENTAR_TRIPLET_H
#define MEMENTAR_TRIPLET_H

#include <iostream>
#include <regex>
#include <string>
#include <vector>

namespace mementar {

  class Triplet
  {
  public:
    Triplet(const std::string& subject, const std::string& predicat, const std::string& object, bool add = true) : subject_(subject),
                                                                                                                   predicat_(predicat),
                                                                                                                   object_(object),
                                                                                                                   add_(add)
    {}

    Triplet(const std::string& str_triplet = "", bool add = true)
    {
      std::vector<std::string> splitted = split(str_triplet, "|");
      if(splitted.empty() == false)
        subject_ = splitted[0];
      if(splitted.size() >= 2)
        predicat_ = splitted[1];
      if(splitted.size() >= 3)
        object_ = splitted[2];
      add_ = add;
    }

    static std::string serialize(const Triplet& triplet)
    {
      return (triplet.add_ ? "A|" : "D|") + triplet.subject_ + "|" + triplet.predicat_ + "|" + triplet.object_;
    }

    static std::string serialize(const Triplet* triplet)
    {
      return (triplet->add_ ? "A|" : "D|") + triplet->subject_ + "|" + triplet->predicat_ + "|" + triplet->object_;
    }

    static Triplet deserialize(const std::string& str)
    {
      if(std::regex_match(str, match, regex))
        return Triplet(match[2].str(), match[3].str(), match[4].str(), match[1].str() == "A");
      else if(std::regex_match(str, match, regex2))
        return Triplet(match[2].str(), match[3].str(), match[4].str(), (match[1].str() == "ADD") || (match[1].str() == "add"));
      else
        return Triplet();
    }

    static Triplet* deserializePtr(const std::string& str)
    {
      if(std::regex_match(str, match, regex))
        return new Triplet(match[2].str(), match[3].str(), match[4].str(), match[1].str() == "A");
      else if(std::regex_match(str, match, regex2))
        return new Triplet(match[2].str(), match[3].str(), match[4].str(), (match[1].str() == "ADD") || (match[1].str() == "add"));
      else
        return nullptr;
    }

    bool valid() const
    {
      return ((subject_.empty() == false) &&
              (predicat_.empty() == false) &&
              (object_.empty() == false));
    }

    std::string toString() const
    {
      return (add_ ? "[add]" : "[del]") + subject_ + "|" + predicat_ + "|" + object_;
    }

    bool operator==(const Triplet& other) const
    {
      return ((add_ == other.add_) && (subject_ == other.subject_) && (predicat_ == other.predicat_) && (object_ == other.object_));
    }

    bool operator==(const Triplet* other) const
    {
      return ((add_ == other->add_) && (subject_ == other->subject_) && (predicat_ == other->predicat_) && (object_ == other->object_));
    }

    bool fit(const Triplet& other) const
    {
      return ((add_ == other.add_) && (subject_ == other.subject_) && (predicat_ == other.predicat_) && (object_ == other.object_));
    }

    friend std::ostream& operator<<(std::ostream& os, const Triplet& triplet)
    {
      os << triplet.toString();
      return os;
    }

    std::string subject_;
    std::string predicat_;
    std::string object_;
    bool add_;

  protected:
    static std::regex regex;
    static std::regex regex2;
    static std::smatch match;

  private:
    std::vector<std::string> split(const std::string& str, const std::string& delim)
    {
      std::vector<std::string> tokens;
      size_t prev = 0, pos = 0;
      do
      {
        pos = str.find(delim, prev);
        if(pos == std::string::npos)
          pos = str.length();

        std::string token = str.substr(prev, pos - prev);

        if(!token.empty())
          tokens.push_back(token);
        prev = pos + delim.length();
      } while((pos < str.length()) && (prev < str.length()));

      return tokens;
    }
  };

} // namespace mementar

#endif // MEMENTAR_TRIPLET_H
