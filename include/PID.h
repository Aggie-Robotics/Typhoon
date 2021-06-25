//
// Created by Aaron Quigg on 6/24/2021.
//

#ifndef WORLDS_PID_H
#define WORLDS_PID_H

#include "main.h"

#include <utility>

namespace SpencerPID{
    struct PIDOut{
        double power;
        double error;
    };

    class PID {
    private:
        double target;
        std::function<double()> current_function;
        double last_time;
        double totalError;
        double last_error;
        double max_i;
        double p;
        double i;
        double d;

    public:
        PID(double target, std::function<double()> current_function, double p, double i, double d, double max_i) :
                target(target), current_function(std::move(current_function)), last_time(0.0), totalError(0.0),
                last_error(target - this->current_function()), max_i(std::abs(max_i)), p(p), i(i), d(d){}

        PIDOut run(){
            auto now_error = current_function() - target;

            double d_value = 0;
            if(last_time != 0){
                auto now_time = (double)pros::micros() / 1000000.0;
                auto elapsed_time = now_time - last_time;
                last_time = now_time;

                totalError += now_error * i * (double)elapsed_time;

                totalError = std::max(std::min(totalError, max_i), -max_i);

                if(elapsed_time != 0){
                    d_value = (now_error - last_error) * d / elapsed_time;
                }
            }
            else{
                last_time = (double)pros::micros() / 1000000.0;
            }
            last_error = now_error;

            PIDOut out{};
            out.error = std::abs(now_error);
            out.power = now_error * p + totalError + d_value;
            return out;
        }
    };
}

#endif //WORLDS_PID_H
