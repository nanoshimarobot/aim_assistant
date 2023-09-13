/*
 * low_pass_filter.h
 *
 *  Created on: Aug 23, 2021
 *      Author: ishihara
 */

#pragma once

namespace abu2023 {
namespace cpp_general {

class Filter {
public:
  virtual double filtering(double input) = 0;
  virtual void reset(void) = 0;
};

class LowPassFilter : public Filter {
  double lpf_gain;
  double before_output_;

public:
  LowPassFilter(double time_constant, double control_freq)
      : lpf_gain((1. / control_freq) / ((1. / control_freq) + time_constant)), before_output_(0) {}
  double filtering(double input) {
    double output = before_output_ + lpf_gain * (input - before_output_);
    before_output_ = output;
    return output;
  }
  double get() { return before_output_; }
  void reset(void) { reset(0); }
  void reset(double u) { before_output_ = u; }
};

} // namespace cpp_general
} // namespace abu2023