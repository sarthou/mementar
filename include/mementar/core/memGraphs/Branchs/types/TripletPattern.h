#ifndef MEMENTAR_TRIPLETPATTERN_H
#define MEMENTAR_TRIPLETPATTERN_H

#include <ostream>
#include <string>

#include "mementar/core/memGraphs/Branchs/types/Triplet.h"

namespace mementar {

  class TripletPattern : public Triplet
  {
  public:
    TripletPattern(const std::string& subject,
                   const std::string& predicat,
                   const std::string& object,
                   bool add) : Triplet(subject, predicat, object, add),
                               operator_is_undefined_(false)
    {
      init();
    }

    TripletPattern(const std::string& subject,
                   const std::string& predicat,
                   const std::string& object) : Triplet(subject, predicat, object, true),
                                                operator_is_undefined_(true)

    {
      init();
    }

    TripletPattern(const std::string& str_triplet, bool add) : Triplet(str_triplet, add),
                                                               operator_is_undefined_(false)
    {
      init();
    }

    explicit TripletPattern(const std::string& str_triplet) : Triplet(str_triplet, true),
                                                              operator_is_undefined_(true)
    {
      init();
    }

    explicit TripletPattern(const Triplet& triplet) : Triplet(triplet)
    {
      init();
    }

    TripletPattern(const TripletPattern& other) = default;

    static TripletPattern deserialize(const std::string& str)
    {
      if(std::regex_match(str, match, regex))
      {
        if(match[1].str() == "?")
          return TripletPattern(match[2].str(), match[3].str(), match[4].str());
        else
          return TripletPattern(match[2].str(), match[3].str(), match[4].str(), match[1].str() == "A");
      }
      else if(std::regex_match(str, match, regex2))
      {
        if(match[1].str() == "?")
          return TripletPattern(match[2].str(), match[3].str(), match[4].str());
        else
          return TripletPattern(match[2].str(), match[3].str(), match[4].str(), (match[1].str() == "ADD") || (match[1].str() == "add"));
      }
      else
        return TripletPattern(Triplet());
    }

    static std::string serialize(const TripletPattern& pattern)
    {
      return Triplet::serialize(pattern);
    }

    static std::string serialize(const TripletPattern* pattern)
    {
      return Triplet::serialize(pattern);
    }

    std::string toString() const
    {
      if(operator_is_undefined_ == false)
        return Triplet::toString();
      else
        return "[?]" + subject_ + "|" + predicat_ + "|" + object_;
    }

    bool fit(const Triplet& other) const
    {
      return (((add_ == other.add_) || operator_is_undefined_) && ((subject_ == other.subject_) || subject_is_undefined_) && ((predicat_ == other.predicat_) || predicat_is_undefined_) && ((object_ == other.object_) || object_is_undefined_));
    }

    bool operator==(const Triplet& other) const = delete;
    bool operator==(const Triplet* other) const = delete;

    void setSubjectAsClass() { subject_is_indiv_ = false; }
    void setSubjectAsIndividual() { subject_is_indiv_ = true; }
    void setObjectAsClass() { object_is_indiv_ = false; }
    void setObjectAsIndividual() { object_is_indiv_ = true; }
    void setPredicatAsDataProperty() { predicat_is_object_property_ = false; }
    void setPredicatAsObjectProperty() { predicat_is_object_property_ = true; }

    bool isSubjectClass() const { return !subject_is_indiv_; }
    bool isSubjectIndividual() const { return subject_is_indiv_; }
    bool isObjectClass() const { return !object_is_indiv_; }
    bool isObjectIndividual() const { return object_is_indiv_; }
    bool isPredicatDataProperty() const { return !predicat_is_object_property_; }
    bool isPredicatObjectProperty() const { return predicat_is_object_property_; }

    bool isOperatorUndefined() const { return operator_is_undefined_; }
    bool isSubjectUndefined() const { return subject_is_undefined_; }
    bool isObjectUndefined() const { return object_is_undefined_; }
    bool isPredicatUndefined() const { return predicat_is_undefined_; }

  protected:
    bool subject_is_indiv_;
    bool object_is_indiv_;
    bool predicat_is_object_property_;

    bool operator_is_undefined_;
    bool subject_is_undefined_;
    bool object_is_undefined_;
    bool predicat_is_undefined_;

  private:
    void init()
    {
      subject_is_indiv_ = true;
      object_is_indiv_ = true;
      predicat_is_object_property_ = true;

      subject_is_undefined_ = (this->subject_ == "?");
      object_is_undefined_ = (this->object_ == "?");
      predicat_is_undefined_ = (this->predicat_ == "?");
    }
  };

} // namespace mementar

#endif // MEMENTAR_TRIPLETPATTERN_H
