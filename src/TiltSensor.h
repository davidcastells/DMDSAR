/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TiltSensor.h
 * Author: dcr
 *
 * Created on January 12, 2018, 6:18 PM
 */

#ifndef TILTSENSOR_H
#define TILTSENSOR_H

#include "AbstractIMU.h"

class TiltSensor {
public:
    TiltSensor(AbstractIMU* imu);
    virtual ~TiltSensor();
    
public:
    void getTilt(double* roll, double* pitch, double* yaw);
    
private:
    AbstractIMU* m_pImu;
    
    int m_i;
    double m_ax[10];
    double m_ay[10];
    double m_az[10];
    
};

#endif /* TILTSENSOR_H */

