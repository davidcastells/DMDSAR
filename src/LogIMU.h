/*
 * Copyright (C) 2017 Universitat Autonoma de Barcelona - David Castells-Rufas <david.castells@uab.cat>
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
 * File:   LogIMU.h
 * Author: dcr
 *
 * Created on January 17, 2018, 12:07 PM
 */

#ifndef LOGIMU_H
#define LOGIMU_H

#include "AbstractIMU.h"
#include "PerformanceLap.h"

class LogIMU : public AbstractIMU
{
public:
    LogIMU();
    LogIMU(const LogIMU& orig);
    virtual ~LogIMU();

public:
    void getAcceleration(double* ax, double *ay, double* az);
    void getGyroscope(double* gx, double *gy, double* gz);
    void getMagnetometer(double* mx, double *my, double* mz);
    
private:
    void readHeader();
    void readLine();
    
private:
    FILE* m_fp;
    long m_offset;      // file offset of the first line with data
    
    double timestamp;
    double xacc;
    double yacc;
    double zacc;
    double xr;
    double yr;
    double zr;
    double xm;
    double ym;
    double zm;
    
    PerformanceLap m_lap;
    double m_realTimeInitialReference;  // real time reference from the first sample
    double m_logTimeInitialReference;   // log time reference from the first sample
};

#endif /* LOGIMU_H */

