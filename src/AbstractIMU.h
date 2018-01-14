/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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
    
private:
    
};

#endif /* ABSTRACTIMU_H */

