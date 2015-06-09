#ifndef BATTEKF_H
#define BATTEKF_H

#include "../AP_GenEKF/ekfilter.h"
#include <AP_HAL.h>

class BattEKF : public Kalman::EKFilter<double,1,false,true,false> {
public:
    BattEKF();

protected:
    void makeBaseA();
    void makeBaseW();
    void makeBaseV();
    void makeBaseR();

    void makeH();
    void makeQ();
    void makeProcess();
    void makeMeasure();

    float _prev_time;
};

typedef BattEKF::Vector Vector;
typedef BattEKF::Matrix Matrix;

#endif
