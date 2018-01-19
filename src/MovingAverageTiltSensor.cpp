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
 * File:   MovingAverageTiltSensor.cpp
 * Author: dcr
 * 
 * Created on January 19, 2018, 4:16 PM
 */

#include "MovingAverageTiltSensor.h"

#include <math.h>

MovingAverageTiltSensor::MovingAverageTiltSensor(AbstractIMU* imu)
{
    m_pImu = imu;
    
    m_i = 0;
}



MovingAverageTiltSensor::~MovingAverageTiltSensor()
{
}

void MovingAverageTiltSensor::getTilt(double* roll, double* pitch, double* yaw)
{
    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;
    m_pImu->getAcceleration(&ax, &ay, &az);
    m_pImu->getGyroscope(&gx, &gy, &gz);
    m_pImu->getMagnetometer(&mx, &my, &mz);
 
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
    