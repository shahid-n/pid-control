/*
 * pidtuner.cpp
 *
 *  Created on: 10.08.2018
 *      Author: academy
 */

#include "pid_tuner.h"
#include <iostream>

#include "PID.h"

pidTuner::pidTuner(PID& pid_steering, PID& pid_throttle, double tolerance)
  : pid_steering_(pid_steering),
    pid_throttle_(pid_throttle),
    tolerance_(tolerance),
    ran_for_steps_(0),
    iterations_(0),
    params_ { pid_steering.Kp, pid_steering.Ki, pid_steering.Kd, pid_throttle.Kp, pid_throttle.Ki, pid_throttle.Kd},
    d_params_ { 1, 1, 1, 0.0, 0.0, 0.0},
    best_err_(-1),
    curr_error_(0),
    twiddle_state_(START),
    twiddle_param_(0),
    error_(0),
    ignore_initial_steps(100),
    offTrack_(false) {
}

pidTuner::~pidTuner() {
}

bool pidTuner::hasFinishedRun() {
  ran_for_steps_ += 1;
  return (3500 < ran_for_steps_);
}

bool pidTuner::isOffTrack(double cte, double speed) {
  // The car is off track if after some initial steps we get very high CTE values or small vehicle speed
  if (ran_for_steps_ > 4 * ignore_initial_steps) {
    offTrack_ = speed < 4 || cte > 6;
    return offTrack_;
  } else {
    return false;
  }
}

void pidTuner::accumulateCTE(double cte) {
  if (ran_for_steps_ > ignore_initial_steps) {
    error_ += cte * cte;
  }
}

double pidTuner::calcAverageError() {
  double averageError = error_ / (ran_for_steps_ - ignore_initial_steps);
  if (offTrack_) {
    // Penalty
    averageError += 1000;
  }
  return averageError;
}

void pidTuner::twiddle() {
  curr_error_ = calcAverageError();


  double sum = 0.0;
  for (auto& n : d_params_) {
    sum += n;
  }
  // We only have to twiddle if the sum of the parameter modification steps is smaller than the tolerance
  if (sum > tolerance_) {
    switch (twiddle_state_) {
      case START: {
        best_err_ = curr_error_;
        params_[twiddle_param_] += d_params_[twiddle_param_];
        twiddle_state_ = INCREMENTING;
        break;
      }
      case INCREMENTING: {
        if (curr_error_ < best_err_) {
          // Incrementing worked well, so we increment even more next time
          best_err_ = curr_error_;
          d_params_[twiddle_param_] *= 1.1;
          goToNextParam();
          params_[twiddle_param_] += d_params_[twiddle_param_];
        } else {
          // otherwise try in the opposite direction
          params_[twiddle_param_] -= 2 * d_params_[twiddle_param_];
          twiddle_state_ = DECREMENTING;
        }
        break;
      }
      case DECREMENTING: {
        if (curr_error_ < best_err_) {
          // Decrementing worked well, so we decrement even more next time
          best_err_ = curr_error_;
          d_params_[twiddle_param_] *= 1.1;
        } else {
          // Otherwise, we reset the current parameter to its original value and reduce the amount of modification
          params_[twiddle_param_] += d_params_[twiddle_param_];
          d_params_[twiddle_param_] *= 0.9;
        }
        twiddle_state_ = INCREMENTING;
        goToNextParam();
        params_[twiddle_param_] += d_params_[twiddle_param_];
        break;
      }
    }
    pid_steering_.Init(params_[0], params_[1], params_[2]);
    pid_throttle_.Init(params_[3], params_[4], params_[5]);
  }
  offTrack_ = false;
  ran_for_steps_ = 0;
  error_ = 0;
}

void pidTuner::goToNextParam() {
  do {
    twiddle_param_ = (twiddle_param_ + 1) % params_.size();
    // Continue until there is a d_param that we want to change (>0)
  } while (d_params_[twiddle_param_] == 0);

  if (twiddle_param_ == 0) {
    ++iterations_;
  }
}

void pidTuner::print() {
  std::cout << "Iteration: " << iterations_ << " Best Error: " << best_err_
            << " Current Error: " << curr_error_ << " Current Parameter: "
            << twiddle_param_ << " Current State: " << twiddle_state_
            << std::endl;
  std::cout << "Next steering parameters: " << params_[0] << ", " << params_[1] << ", "
            << params_[2]<< std::endl;
  std::cout << "Next throttle parameters: " << params_[3] << ", " << params_[4] << ", "
              << params_[5]<< std::endl;
}
