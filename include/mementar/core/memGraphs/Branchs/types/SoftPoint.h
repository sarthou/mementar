#ifndef MEMENTAR_SOFTPOINT_H
#define MEMENTAR_SOFTPOINT_H

#include <experimental/optional>
#include <cstdlib>

namespace mementar {

class SoftPoint
{
public:
  typedef size_t Ttime;
  static Ttime default_time;
  //typedef float Ttime;

  SoftPoint(Ttime t_start, std::experimental::optional<Ttime> t_end = std::experimental::nullopt)
  {
    t_start_ = t_start;
    t_end_ = t_end;
    t_ = t_start_ + (t_end_.value_or(t_start_) - t_start_) / 2;
  }

  SoftPoint(const SoftPoint& other)
  {
    t_start_ = other.t_start_;
    t_end_ = other.t_end_;
    t_ = t_start_ + (t_end_.value_or(t_start_) - t_start_) / 2;
  }

  SoftPoint(const SoftPoint* other)
  {
    t_start_ = other->t_start_;
    t_end_ = other->t_end_;
    t_ = t_start_ + (t_end_.value_or(t_start_) - t_start_) / 2;
  }

  bool isInstantaneous() const { return t_end_ == std::experimental::nullopt; }
  Ttime getTime() const { return t_; }
  Ttime getTimeStart() const { return t_start_; }
  Ttime getTimeEnd() const { return t_end_.value_or(t_start_); }
  Ttime getTransitionDuration() const { return t_end_.value_or(t_start_) - t_start_; }

  std::string toString() const { return "[" + std::to_string(t_start_) + std::string(t_end_ ? "," + std::to_string(t_end_.value()) : "") + "]"; }


protected:
  Ttime t_start_;
  std::experimental::optional<Ttime> t_end_;
  Ttime t_;
};

} // namespace mementar

#endif // MEMENTAR_SOFTPOINT_H
