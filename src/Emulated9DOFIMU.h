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
 * File:   Emulated9DOFIMU.h
 * Author: dcr
 *
 * Created on January 12, 2018, 6:19 PM
 */

#ifndef EMULATED9DOFIMU_H
#define EMULATED9DOFIMU_H

#include "AbstractIMU.h"

#include <stdlib.h>

class Emulated9DOFIMU : public AbstractIMU 
{
public:
    Emulated9DOFIMU();
    Emulated9DOFIMU(const Emulated9DOFIMU& orig);
    virtual ~Emulated9DOFIMU();

public:
    void getAcceleration(double* ax, double *ay, double* az);
    void getGyroscope(double* gx, double *gy, double* gz);
    void getMagnetometer(double* mx, double *my, double* mz);

protected:
    double getRand(double min, double max);

public:
    double m_accX;
    double m_accY;
    double m_accZ;
    double m_accNoiseX;
    double m_accNoiseY;
    double m_accNoiseZ;
    double m_accDriftX;
    double m_accDriftY;
    double m_accDriftZ;
};

#endif /* EMULATED9DOFIMU_H */

