/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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

