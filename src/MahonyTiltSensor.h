/*
 * Copyright (C) 2018 Universitat Autonoma de Barcelona - David Castells-Rufas <david.castells@uab.cat>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/* 
 * File:   MahonyTiltSensor.h
 * Author: dcr
 *
 * Created on January 19, 2018, 4:08 PM
 */

#ifndef MAHONYTILTSENSOR_H
#define MAHONYTILTSENSOR_H

#include "AbstractTiltSensor.h"

class MahonyTiltSensor : public AbstractTiltSensor
{
public:
    MahonyTiltSensor(AbstractIMU* imu);
    virtual ~MahonyTiltSensor();
   
public:
    virtual void getTilt(double* roll, double* pitch, double* yaw);
    
protected:
    void mahonyUpdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double elapsedTime);
    void mahonyUpdateIMU(double gx, double gy, double gz, double ax, double ay, double az, double elapsedTime);
    void toEulerAngles(double* roll, double* pitch, double* yaw);
    double invSqrt(double x);

private:

    // Quaternion
    double q0;
    double q1;
    double q2;
    double q3;
    
    double integralFBx;
    double integralFBy;
    double integralFBz;
    
    double Ki;
    double Kp;
};

#endif /* MAHONYTILTSENSOR_H */

