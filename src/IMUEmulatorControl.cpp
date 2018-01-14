/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/*
 * File:   IMUEmulatorControl.cpp
 * Author: dcr
 *
 * Created on January 12, 2018, 1:31 PM
 */

#include "IMUEmulatorControl.h"

IMUEmulatorControl::IMUEmulatorControl()
{
    widget.setupUi(this);
}

IMUEmulatorControl::~IMUEmulatorControl()
{
}


void IMUEmulatorControl::setIMU(Emulated9DOFIMU* imu)
{
    m_pImu = imu;
}

void IMUEmulatorControl::init()
{
    connect(widget.btnApply, SIGNAL(pressed()), this, SLOT(onApply()));
}

void IMUEmulatorControl::closeEvent (QCloseEvent *event)
{
    exit(0);
}

void IMUEmulatorControl::onApply()
{
    // ui->lineEdit->text().toStdString()
    std::string sAccX = widget.txtAccX->text().toStdString();
    std::string sAccY = widget.txtAccY->text().toStdString();
    std::string sAccZ = widget.txtAccZ->text().toStdString();
    std::string sAccNoiseX = widget.txtAccNoiseX->text().toStdString();
    std::string sAccNoiseY = widget.txtAccNoiseY->text().toStdString();
    std::string sAccNoiseZ = widget.txtAccNoiseZ->text().toStdString();
    std::string sAccDriftX = widget.txtAccDriftX->text().toStdString();
    std::string sAccDriftY = widget.txtAccDriftY->text().toStdString();
    std::string sAccDriftZ = widget.txtAccDriftZ->text().toStdString();
    
    m_pImu->m_accX = atof(sAccX.c_str());
    m_pImu->m_accY = atof(sAccY.c_str());
    m_pImu->m_accZ = atof(sAccZ.c_str());
    m_pImu->m_accNoiseX = atof(sAccNoiseX.c_str());
    m_pImu->m_accNoiseY = atof(sAccNoiseY.c_str());
    m_pImu->m_accNoiseZ = atof(sAccNoiseZ.c_str());
    m_pImu->m_accDriftX = atof(sAccDriftX.c_str());
    m_pImu->m_accDriftY = atof(sAccDriftY.c_str());
    m_pImu->m_accDriftZ = atof(sAccDriftZ.c_str());
}