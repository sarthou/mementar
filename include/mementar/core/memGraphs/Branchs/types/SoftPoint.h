#ifndef MEMENTAR_SOFTPOINT_H
#define MEMENTAR_SOFTPOINT_H

#include <cstdlib>
#include <optional>
#include <string>

namespace mementar {

  class SoftPoint
  {
  public:
    using Ttime = double;
    static Ttime default_time;
    // typedef float Ttime;

    // Avoid to set it explicit
    SoftPoint(Ttime t_start, std::optional<Ttime> t_end = std::nullopt) : t_start_(t_start),
                                                                          t_end_(t_end),
                                                                          t_(t_start_ + (t_end_.value_or(t_start_) - t_start_) / 2)
    {}

    explicit SoftPoint(const std::string& t_start) : t_start_(std::stod(t_start)),
                                                     t_end_(std::nullopt),
                                                     t_(t_start_)
    {}

    SoftPoint(const std::string& t_start, const std::string& t_end) : t_start_(std::stod(t_start)),
                                                                      t_end_(std::stod(t_end)),
                                                                      t_(t_start_ + (t_end_.value() - t_start_) / 2)
    {}

    SoftPoint(const SoftPoint& other) : t_start_(other.t_start_),
                                        t_end_(other.t_end_),
                                        t_(t_start_ + (t_end_.value_or(t_start_) - t_start_) / 2)
    {}

    explicit SoftPoint(const SoftPoint* other) : t_start_(other->t_start_),
                                                 t_end_(other->t_end_),
                                                 t_(t_start_ + (t_end_.value_or(t_start_) - t_start_) / 2)
    {}

    SoftPoint& operator=(const SoftPoint& other) = default;

    bool isInstantaneous() const { return t_end_ == std::nullopt; }
    Ttime getTime() const { return t_; }
    Ttime getTimeStart() const { return t_start_; }
    Ttime getTimeEnd() const { return t_end_.value_or(t_start_); }
    Ttime getTransitionDuration() const { return t_end_.value_or(t_start_) - t_start_; }

    std::string toString() const { return "[" + std::to_string(t_start_) + std::string(t_end_ ? "," + std::to_string(t_end_.value()) : "") + "]"; }

  protected:
    Ttime t_start_;
    std::optional<Ttime> t_end_;
    Ttime t_;
  };

} // namespace mementar

#endif // MEMENTAR_SOFTPOINT_H
