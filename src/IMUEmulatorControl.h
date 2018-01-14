/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
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
