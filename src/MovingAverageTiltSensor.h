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
 * File:   MovingAverageTiltSensor.h
 * Author: dcr
 *
 * Created on January 19, 2018, 4:16 PM
 */

#ifndef MOVINGAVERAGETILTSENSOR_H
#define MOVINGAVERAGETILTSENSOR_H

#include "AbstractTiltSensor.h"

class MovingAverageTiltSensor : public AbstractTiltSensor
{
public:
    MovingAverageTiltSensor(AbstractIMU* imu);
    virtual ~MovingAverageTiltSensor();
    
public:
    virtual void getTilt(double* roll, double* pitch, double* yaw);
    
private:
    // for moving average filter
    int m_i;
    double m_ax[10];
    double m_ay[10];
    double m_az[10];
};

#endif /* MOVINGAVERAGETILTSENSOR_H */

