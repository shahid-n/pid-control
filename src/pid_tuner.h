/*
 * PIDTuner.h
 *
 *  Created on: 10.08.2018
 *      Author: academy
 */

#ifndef SRC_PIDTUNER_H_
#define SRC_PIDTUNER_H_

#include <vector>
#include "PID.h"

enum twiddleState { START, INCREMENTING, DECREMENTING };

class pidTuner {
  public:
    pidTuner(PID& pid_steering, PID& pid_throttle, double tolerance);
    virtual ~pidTuner();
    bool hasFinishedRun();
    bool isOffTrack(double cte, double speed);
    void accumulateCTE(double cte);
    void twiddle();
    void print();
  private:
    double calcAverageError();
    void goToNextParam();
    PID& pid_steering_, pid_throttle_;
    double tolerance_;
    int ran_for_steps_;
    int iterations_;
    std::vector<double> params_;
    std::vector<double> d_params_;
    double best_err_;
    double curr_error_;
    twiddleState twiddle_state_;
    int twiddle_param_;
    double error_;
    int ignore_initial_steps;
    bool offTrack_;
};

#endif /* SRC_PIDTUNER_H_ */
