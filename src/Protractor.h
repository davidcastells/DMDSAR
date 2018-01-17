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
    bool    doLogIMU;
    
    PerformanceLap m_lastEulerAnglesReport;
};

#endif /* PROTRACTOR_H */

