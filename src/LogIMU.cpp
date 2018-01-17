/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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
