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