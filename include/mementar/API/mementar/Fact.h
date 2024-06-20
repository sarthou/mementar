#ifndef MEMENTAR_API_FACT_H
#define MEMENTAR_API_FACT_H

#include <regex>
#include <string>
#include <vector>

namespace mementar {

  class Fact
  {
  public:
    Fact(const std::string& fact, bool add = true)
    {
      std::smatch match;
      std::regex regex("\\[(\\w+)\\]([^|]+)\\|([^|]+)\\|([^|]+)");
      if(std::regex_match(fact, match, regex))
      {
        subject_ = match[2].str();
        predicat_ = match[3].str();
        object_ = match[4].str();
        add_ = (match[1].str() == "ADD") || (match[1].str() == "add");
      }
      else
      {
        std::vector<std::string> splitted = split(fact, "|");
        if(splitted.empty() == false)
          subject_ = splitted[0];
        if(splitted.size() >= 2)
          predicat_ = splitted[1];
        if(splitted.size() >= 3)
          object_ = splitted[2];
        add_ = add;
      }
    }

    Fact(const std::string& subject,
         const std::string& predicat,
         const std::string& object,
         bool add = true) : subject_(subject),
                            predicat_(predicat),
                            object_(object),
                            add_(add)
    {}

    std::string getSubject() const { return subject_; }
    std::string getPredicat() const { return predicat_; }
    std::string getObject() const { return object_; }
    bool getAdd() const { return add_; }
    std::string operator()() const { return (add_ ? "[add]" : "[del]") + subject_ + "|" + predicat_ + "|" + object_; }
    std::string to_string() const { return (add_ ? "[add]" : "[del]") + subject_ + "|" + predicat_ + "|" + object_; }

    void setSubject(const std::string& subject) { subject_ = subject; }
    void setPredicat(const std::string& predicat) { predicat_ = predicat; }
    void setObject(const std::string& object) { object_ = object; }
    void setAdd(bool add) { add_ = add; }
    void operator()(const std::string& fact, bool add = true)
    {
      std::vector<std::string> splitted = split(fact, "|");
      if(splitted.empty() == false)
        subject_ = splitted[0];
      if(splitted.size() >= 2)
        predicat_ = splitted[1];
      if(splitted.size() >= 3)
        object_ = splitted[2];
      add_ = add;
    }

  private:
    std::string subject_;
    std::string predicat_;
    std::string object_;
    bool add_;

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

#endif // MEMENTAR_API_FACT_H
