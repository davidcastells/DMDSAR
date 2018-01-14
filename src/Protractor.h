/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   Protractor.h
 * Author: dcr
 *
 * Created on January 11, 2018, 5:50 PM
 */

#ifndef PROTRACTOR_H
#define PROTRACTOR_H

#include <QWidget>          // Needed for basic window handling
#include <QtGui>            // Needed for painter

#include "IMUEmulatorControl.h"
#include "AbstractIMU.h"
#include "TiltSensor.h"
#include "PerformanceLap.h"

class Protractor : public QWidget
{
public:
    Protractor(QWidget *parent = 0);
    
    virtual ~Protractor();
    
public:
    void parseOptions(int argc, char* args[]);
    void updateCompass();
    void printUsage();

protected:
    void paintEvent(QPaintEvent *event);
    
private:
    IMUEmulatorControl imuEmulator;
    AbstractIMU*         m_imu;
    TiltSensor*          m_tiltSensor;
        
    double roll;		// lateral (wing) rotation
    double pitch;		// front/back rotation
    double yaw;		// steady plane rotation
    
    
    // options
    bool    doHelp;
    bool    doEmulateIMU;
    
    PerformanceLap m_lastEulerAnglesReport;
};

#endif /* PROTRACTOR_H */

