/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/
#include "common/LowPassFilter.h"
#include <math.h>
#include <iostream>
LPFilter::LPFilter(double samplePeriod, double cutFrequency){
    _weight = 1.0 / ( 1.0 + 1.0/(2.0*M_PI * samplePeriod * cutFrequency) );
    // std::cout << "LPFilter weight: " << _weight << std::endl;
    _start  = false;
}

void LPFilter::addValue(double newValue){
    if(!_start){
        _start = true;
        _pastValue = newValue;
    }
    _pastValue = _weight*newValue + (1-_weight)*_pastValue;
}

double LPFilter::getValue(){
    return _pastValue;
}

void LPFilter::clear(){
    _start = false;
}

