#pragma once

#include <map>
#include <vector>

#include "type.h"

namespace cityfly::common {

template <typename value_t> using TimeValueMap = std::map<Time, value_t>;

// for self-defined value_t, implement your own Trait
template <typename value_t> struct InterpolaterTrait {
  static value_t interpolate(const value_t &x1, const value_t &x2, Time t1,
                             Time t2, Time t) {
    return x1 + (x2 - x1) * (scalar_t(t - t1) / scalar_t(t2 - t1));
  }

  static value_t extrapolate(const value_t &x1, const value_t &x2, Time t1,
                             Time t2, Time t) {
    return x2 + (x2 - x1) * (scalar_t(t - t2) / scalar_t(t2 - t1));
  }
};

template <typename value_t> class Interpolater {
public:
  Interpolater(Time max_age, Time max_predict)
      : max_age_(max_age), max_predict_(max_predict) {}

  int put(Time t, const value_t &val) {
    time_value_[t] = val;
    removeValuesBefore(endTime() - max_age_);
    return time_value_.size();
  }

  bool
  get(Time t, value_t *val,
      typename TimeValueMap<value_t>::const_iterator *prev = nullptr,
      typename TimeValueMap<value_t>::const_iterator *next = nullptr) const {
    // 0 element
    if (time_value_.empty()) {
      return false;
    }

    auto it = time_value_.find(t);
    if (it != time_value_.end()) {
      if (val) {
        *val = it->second;
      }
      if (prev) {
        *prev = it;
      }
      if (next) {
        *next = std::next(it);
      }
      return true;
    }

    // 1 element
    if (time_value_.size() <= 1) {
      return false;
    }

    // 2 and more element
    if (t < startTime() || t > endTime() + max_predict_) {
      return false;
    }

    auto upper = time_value_.upper_bound(t);
    auto lower = std::prev(upper);

    if (val) {
      if (upper == time_value_.end()) { // extrapolate
        auto lower2 = std::prev(lower);
        *val = InterpolaterTrait<value_t>::extrapolate(
            lower2->second, lower->second, lower2->first, lower->first, t);
      } else { // interpolate
        *val = InterpolaterTrait<value_t>::interpolate(
            lower->second, upper->second, lower->first, upper->first, t);
      }
    }

    if (prev) {
      *prev = lower;
    }

    if (next) {
      *next = upper;
    }

    return true;
  }

  bool getRange(Time t0, Time t1, std::vector<value_t> *vals) {
    value_t start, end;
    typename TimeValueMap<value_t>::const_iterator t0_next;
    typename TimeValueMap<value_t>::const_iterator t1_next;

    if (!get(t0, &start, nullptr, &t0_next)) {
      return false;
    }

    if (!get(t1, &end, nullptr, &t1_next)) {
      return false;
    }

    vals->reserve(std::distance(t0_next, t1_next) + 1);
    vals->clear();

    vals->emplace_back(start);
    for (auto it = t0_next; it->first < t1; ++it) {
      vals->emplace_back(it->second);
    }
    vals->emplace_back(end);

    return true;
  }

  void clear() { time_value_.clear(); }

private:
  Time startTime() const { return time_value_.cbegin()->first; }

  Time endTime() const { return time_value_.crbegin()->first; }

  void removeValuesBefore(Time t) {
    auto it = time_value_.begin();
    while (it != time_value_.end() && it->first < t) {
      it = time_value_.erase(it);
    }
  }

  Time max_age_;
  Time max_predict_;
  TimeValueMap<value_t> time_value_;
};

} // namespace cityfly::common
