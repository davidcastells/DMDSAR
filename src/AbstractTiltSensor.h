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
 * File:   TiltSensor.h
 * Author: dcr
 *
 * Created on January 12, 2018, 6:18 PM
 */

#ifndef TILTSENSOR_H
#define TILTSENSOR_H

#include "AbstractIMU.h"
#include "PerformanceLap.h"

class AbstractTiltSensor 
{
public:
    virtual void getTilt(double* roll, double* pitch, double* yaw) = 0;
    
public:
    bool verbosity;
    
protected:
    AbstractIMU* m_pImu;
    PerformanceLap m_lap;
};

#endif /* TILTSENSOR_H */

