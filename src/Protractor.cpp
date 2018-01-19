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
 * File:   Protractor.cpp
 * Author: dcr
 * 
 * Created on January 11, 2018, 5:50 PM
 */

#include "Protractor.h"

#include "Emulated9DOFIMU.h"
#include "LogIMU.h"
#include "MPU9250.h"
#include "MovingAverageTiltSensor.h"
#include "MahonyTiltSensor.h"

#include <math.h>

#define PI  3.141592


Protractor::Protractor(QWidget *parent) : QWidget(parent)
{
    m_lastEulerAnglesReport.start();
    
    setWindowTitle(tr("Protractor"));

    // Variable to hold the flags applicable to windows 
    Qt::WindowFlags flags = 0;         // default value for QWidget window
    flags |= Qt::FramelessWindowHint;  // turn window frameless
    setWindowFlags(flags);

    setStyleSheet("background-color: black;");

    // Set window size to cover all the TI DLP LightCrafter Display 2000 EVM 
    resize(640, 360);
    move(0,0);  // Default position for the window is top left


    // Provides clock time functions for the current object and customizes the C++ 
    // operators for operands of QTimer type. 
    QTimer *timer = new QTimer(this);
  
    // Setup the standard update() slot so that the flight indicator face is 
    // updated when the timer emits the timeout() signal. */
    connect(timer, SIGNAL(timeout()), this, SLOT(update()));
    timer->start(10);  // 10 milisecond timer

  /* ---------------------------> MPU9250 Setup <---------------------------- */

/*  printf("===== MPU 9250 Demo using Linux =====\n");
  openI2C(&file, 2);  // BeagleBone Black I2C
  myIMU.ptrFile = &file;

  mpu9250Setup();
  
  calibrate();
 */
}


Protractor::~Protractor()
{
    
}

void Protractor::printUsage()
{
    printf("Usage: DMDSAR.exe [options]\n");
    printf("[options]\n");
    printf("   --help           Show this message\n" );
    printf("   --imu=[emulate|log|MPU9250]  \n");
    printf("          emulate - emulates a 9 DOF IMU\n");
    printf("          log - uses the WEAN HALL dataset as input\n");
    printf("                    (expected in ./data/wean_wide_interesting.imu.log)\n");
    printf("          MPU9250 - uses the MPU9250 IMU\n");
    printf("   --tilt=[maf|mahony|madgwick]");
    
}

void Protractor::parseOptions(int argc, char* args[])
{
    
 
    doHelp = false;
    doEmulateIMU = false;
    doLogIMU = false;
    doMPU9250 = false;
    doMaf = false;
    doMahony = false;
    doMadgwick = false;
    doVerbose = false;
    
    for (int i=0; i < argc; i++)
    {
        if (strcmp(args[i], "--help") == 0)
            doHelp = true;
        else if (strncmp(args[i], "--imu=", 6) == 0)
        {
            char* sImuType = &args[i][6];
            
            printf("Selecting IMU %s...\n", sImuType);
            
            if (strcmp(sImuType, "emulate") == 0)
                doEmulateIMU = true;
            else if (strcmp(sImuType, "log") == 0)
                doLogIMU = true;
            else if (strcmp(sImuType, "MPU9250") == 0)
                doMPU9250 = true;
            else
            {
                printf("unsupported IMU\n");
                exit(-1);
            }
        }
        else if (strncmp(args[i], "--tilt=", 7) == 0)
        {
            char* sTiltType = &args[i][7];
            printf("Selecting Tilt Sensor %s...\n", sTiltType);
            
            if (strcmp(sTiltType, "maf") == 0)
                doMaf = true;
            else if (strcmp(sTiltType, "mahony") == 0)
                doMahony = true;
            else if (strcmp(sTiltType, "madgwick") == 0)
                doMadgwick = true;
        }
        else if (strcmp(args[i], "--verbose") == 0)
        {
            doVerbose = true;
        }

    }
    
    if (doHelp)
    {
        printUsage();
        exit(0);
    }
    
    if (doEmulateIMU)
    {
        Emulated9DOFIMU* emulatedImu = new Emulated9DOFIMU();
        
        m_imu = emulatedImu;

        imuEmulator.setIMU(emulatedImu);
        imuEmulator.setModal(false);
        imuEmulator.init();
        imuEmulator.setVisible(true);
    }
    else if (doLogIMU)
    {
        LogIMU* logImu = new LogIMU();
        
        m_imu = logImu;
    }
    else if (doMPU9250)
    {
        MPU9250* mpu = new MPU9250();
        
        m_imu = mpu;
    }
    else
    {
        printf("IMU not selected\n");
        exit(-1);
    }

    if (doMaf)
    {
        m_tiltSensor = new MovingAverageTiltSensor(m_imu);
    }
    else if (doMahony)
    {
        m_tiltSensor = new MahonyTiltSensor(m_imu);
    }
    else if (doMadgwick)
    {
        printf("Madgwick not implemented yet\n");
        exit(-1);
    }
    else
    {
        printf("Tilt Sensor not selected\n");
        exit(-1);
    }


    if (doVerbose)
        m_tiltSensor->verbosity = true;
    
}

void Protractor::updateCompass()
{
    m_tiltSensor->getTilt(&roll, &pitch, &yaw);
    
    m_lastEulerAnglesReport.stop();
    
    if (m_lastEulerAnglesReport.lap() > 2.0)
    {
        printf("roll=%f ", roll);
        printf("pitch=%f ", pitch);
        printf("yaw=%f ", yaw);
        printf("\n");
        m_lastEulerAnglesReport.start();
    }
}

void Protractor::paintEvent(QPaintEvent *event)
{
    double degreesPerRad = 180 / PI;
    
    // Aquiring MPU9250 Raw Values 
    updateCompass();
  
	double wallDistance = 0.4; 	// 40 cm
	double yawRad = yaw;
	double displacement = sin(yawRad) * wallDistance; // in meters
	double pixelsPerMeter = 640 / 0.3; 	// empirically observed that the screen at 40cm ocupies 30 cm (using a resolution of 640x480)
	
	int pixelsDisplacement = displacement * pixelsPerMeter;
	
	double pitchRad = pitch;
	double verticalDisplacement = sin(pitchRad) * wallDistance; // in meters
	int verticalPixelsDisplacement = verticalDisplacement * pixelsPerMeter;
	
  /* -----------------------------> Variables <------------------------------ */
//  QTime time = QTime::currentTime();

  char sDegrees[5];
  sprintf(sDegrees, "%d", (int) (roll * degreesPerRad));
  QRect angleText(-30, -30, 60, 60);

  char sPitch[5];
  sprintf(sPitch, "%d", (int) (pitch * degreesPerRad));
  QRect pitchText(-180, -10, 30, 20);
  

  /* -------------------------------> Colors <------------------------------- */
  QColor circleColor(127, 127, 127);
  QColor textColor(255, 255, 255);
  QColor textBackground(127, 127, 127);
  QColor angleColor(0, 127, 127, 191);
  QColor pitchColor(255, 0, 0);

  /* Protractor marks colors */
  QColor tensColor(255,255,250);
  QColor onesColor(227, 231, 231, 191);


  /* -----------------------> Painter Configuration <------------------------ */
  /* Contents of custom widgets are drawn with a QPainter */
  QPainter painter(this);  // Provides painter functions for the current object
  painter.setRenderHint(QPainter::Antialiasing);  // Draw smoother diagonals
  /* Set the center of the window as the origin */
  painter.translate(width() / 2 + pixelsDisplacement, height() / 2 - verticalPixelsDisplacement) ;

  /* -----------------------------> Angle Bar <------------------------------ */
  painter.setPen(angleColor);    // Set border color
  painter.setBrush(angleColor);  // Set fill color
  painter.drawRect(-140, -5, 110, 10);  // x, y, width, height
  painter.drawRect(30, -5, 110, 10);

  /* ************************************************************************ */
  /* Compensate rotation for every object drawn from here */
  /* ************************************************************************ */
  painter.rotate(-roll * degreesPerRad);

  /* ----------------------------> Inner Circle <---------------------------- */
  painter.setPen(circleColor);
  painter.setBrush(circleColor);
  painter.drawEllipse(-30,-30, 60, 60);  // x, y, width, height

  /* --------------------------> Text indicators <--------------------------- */
  /* Draw pitch rectangle indicator */
  painter.setPen(textBackground);
  painter.setBrush(textBackground);
  painter.drawRect(-180, -10, 30, 20);

  /* Draw text */
  painter.setPen(textColor);  // Set text color
  painter.drawText(angleText, Qt::AlignCenter, sDegrees);
  painter.drawText(pitchText, Qt::AlignCenter, sPitch);

  /* -----------------------------> Protractor <----------------------------- */
  painter.setPen(tensColor);
  painter.setBrush(tensColor);
  for (int i = 0; i < 36; ++i) {
    if (i != 0 && (i % 9) != 0) {
      painter.drawLine(120, 0, 140, 0);
    } else {
      painter.drawLine(110, 0, 140, 0);  // 90, 180, 270, 360
    }
    painter.rotate(10.0);
  }

  painter.setPen(onesColor);
  painter.setBrush(onesColor);
  for (int j = 0; j < 360; ++j) {
    if ((j % 10) != 0)
      painter.drawLine(130, 0, 140, 0);
    painter.rotate(1.0);
  }

  /* -----------------------------> Pitch Bar <------------------------------ */
  /* Compensate for pitch */
  painter.translate(0, 0 /*pitch*/);  // x, y

  painter.setPen(pitchColor);
  painter.setBrush(pitchColor);
  painter.drawRect(-140, -5, 110, 10);
  painter.drawRect(30, -5, 110, 10);
}