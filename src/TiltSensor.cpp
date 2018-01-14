/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   TiltSensor.cpp
 * Author: dcr
 * 
 * Created on January 12, 2018, 6:18 PM
 */

#include "TiltSensor.h"

#include <math.h>

TiltSensor::TiltSensor(AbstractIMU* imu)
{
    m_pImu = imu;
    
    m_i = 0;
}

TiltSensor::~TiltSensor()
{
}

void TiltSensor::getTilt(double* roll, double* pitch, double* yaw)
{
    double ax, ay, az;
    m_pImu->getAcceleration(&ax, &ay, &az);
    
    // If filtering enabled
    m_ax[m_i] = ax;
    m_ay[m_i] = ay;
    m_az[m_i] = az;
    
    m_i = (m_i + 1) % 10;
    
    ax = ay = az = 0;
    for (int i=0; i<10; i++)
    {
        ax += m_ax[i];
        ay += m_ay[i];
        az += m_az[i];
    }
    ax /= 10.0;
    ay /= 10.0;
    az /= 10.0;
    // done
    
    *roll = atan2(ay , az);
    *pitch = atan2(ax, sqrt(ay*ay + az*az));
    *yaw = 0;
}
    