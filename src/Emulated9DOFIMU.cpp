/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Emulated9DOFIMU.cpp
 * Author: dcr
 * 
 * Created on January 12, 2018, 6:19 PM
 */

#include "Emulated9DOFIMU.h"

#include <stdio.h>

Emulated9DOFIMU::Emulated9DOFIMU()
{
}

Emulated9DOFIMU::Emulated9DOFIMU(const Emulated9DOFIMU& orig)
{
}

Emulated9DOFIMU::~Emulated9DOFIMU()
{
}

/**
 * 
 * @param min
 * @param max
 * @return a random value between min and max
 */
double Emulated9DOFIMU::getRand(double min, double max)
{
    double range = (max - min);
    double mean = (min + max) / 2.0;
    double rnd = rand();
    rnd = rnd * range;
    rnd = rnd / RAND_MAX;
    
    rnd -= mean;
    
    return rnd;
}

void Emulated9DOFIMU::getAcceleration(double* ax, double *ay, double* az)
{
    *ax = m_accX + getRand(-1, 1) * m_accNoiseX + m_accDriftX;
    *ay = m_accY + getRand(-1, 1) * m_accNoiseY + m_accDriftY;
    *az = m_accZ + getRand(-1, 1) * m_accNoiseZ + m_accDriftZ;
    
    
}