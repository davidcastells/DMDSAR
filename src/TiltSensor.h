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

