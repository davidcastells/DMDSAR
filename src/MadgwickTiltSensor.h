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
 * File:   MadgwickTiltSensor.h
 * Author: dcr
 *
 * Created on January 21, 2018, 9:43 AM
 */

#ifndef MADGWICKTILTSENSOR_H
#define MADGWICKTILTSENSOR_H

#include "AbstractIMU.h"
#include "AbstractTiltSensor.h"


class MadgwickTiltSensor : public AbstractTiltSensor
{
public:
    MadgwickTiltSensor(AbstractIMU* imu);
    virtual ~MadgwickTiltSensor();

public:
    virtual void getTilt(double* roll, double* pitch, double* yaw);

private:
    void update(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double elapsedTime);
    void updateIMU(double gx, double gy, double gz, double ax, double ay, double az, double elapsedTime);
    double invSqrt(double x);
    void toEulerAngles(double* roll, double* pitch, double* yaw);
    
private:
    AbstractIMU* m_pImu;
    
// Quaternion
    double q0;
    double q1;
    double q2;
    double q3;

    double beta;
};

#endif /* MADGWICKTILTSENSOR_H */

