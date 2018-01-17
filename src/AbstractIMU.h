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
 * File:   AbstractIMU.h
 * Author: dcr
 *
 * Created on January 12, 2018, 6:18 PM
 */

#ifndef ABSTRACTIMU_H
#define ABSTRACTIMU_H

class AbstractIMU {
public:
    AbstractIMU();
    AbstractIMU(const AbstractIMU& orig);
    virtual ~AbstractIMU();
    
public:
    /** Return acceleration in m/s^2 units */
    virtual void getAcceleration(double* ax, double *ay, double* az) = 0;
    virtual void getGyroscope(double* gx, double *gy, double* gz) = 0;
    virtual void getMagnetometer(double* mx, double *my, double* mz) = 0;
    
private:
    
};

#endif /* ABSTRACTIMU_H */

