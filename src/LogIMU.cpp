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
 * File:   LogIMU.cpp
 * Author: dcr
 * 
 * Created on January 17, 2018, 12:07 PM
 */

#include <cstdio>
#include <assert.h>

#include "LogIMU.h"

#include "PerformanceLap.h"

LogIMU::LogIMU()
{
    m_fp = fopen("data/wean_wide_interesting.imu.log", "r");
    
    assert(m_fp);
    
    readHeader();
    readLine(); // we get the first time stamp
    m_logTimeInitialReference = timestamp;
    m_realTimeInitialReference = m_lap.dtime();
}

LogIMU::LogIMU(const LogIMU& orig)
{
}

LogIMU::~LogIMU()
{
    fclose(m_fp);
}

void LogIMU::readHeader()
{
    char header[200];
    fgets(header, sizeof(header), m_fp);
    
    m_offset = ftell(m_fp);
}

void LogIMU::readLine()
{
    // WEAN HALL Dataset format
    //    # timestamp xacc yacc zacc xr yr zr xm ym zm t roll pitch yaw
    double t, roll, pitch, yaw; // we discard this data
    
    int v = fscanf(m_fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &timestamp, &xacc, &yacc, &zacc, &xr, &yr, &zr, &xm, &ym, &zm, &t, &roll, &pitch, &yaw);
    
    if (v < 14)
    {
        fseek(m_fp, m_offset, SEEK_SET);
        printf("starting again");
        
        fscanf(m_fp, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", &timestamp, &xacc, &yacc, &zacc, &xr, &yr, &zr, &xm, &ym, &zm, &t, &roll, &pitch, &yaw);
    }
}


void LogIMU::getAcceleration(double* ax, double *ay, double* az)
{
    double realElapsedTime = m_lap.dtime() - m_realTimeInitialReference;
    double logElapsedTime = timestamp - m_logTimeInitialReference;
    
    while (logElapsedTime < realElapsedTime)
    {
        readLine();
        logElapsedTime = timestamp - m_logTimeInitialReference;
    }
    
    *ax = xacc;
    *ay = yacc;
    *az = zacc;
}
    
void LogIMU::getGyroscope(double* gx, double *gy, double* gz)
{
    *gx = xr;
    *gy = yr;
    *gz = zr;
}

void LogIMU::getMagnetometer(double* mx, double *my, double* mz)
{
    *mx = xm;
    *my = ym;
    *mz = zm;
}
