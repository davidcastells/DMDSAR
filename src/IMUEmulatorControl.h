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
 * File:   IMUEmulatorControl.h
 * Author: dcr
 *
 * Created on January 12, 2018, 1:31 PM
 */

#ifndef _IMUEMULATORCONTROL_H
#define _IMUEMULATORCONTROL_H

#include "ui_IMUEmulatorControl.h"
#include "Emulated9DOFIMU.h"

class IMUEmulatorControl : public QDialog 
{
    Q_OBJECT
public:
    IMUEmulatorControl();
    virtual ~IMUEmulatorControl();
    
public:
    void init();
    void setIMU(Emulated9DOFIMU* imu);
    
public slots:
    void onApply();

private:
    void closeEvent(QCloseEvent *event);

private:
    Ui::IMUEmulatorControl widget;
    
    Emulated9DOFIMU* m_pImu;
};

#endif /* _IMUEMULATORCONTROL_H */
