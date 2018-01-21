# This file is generated automatically. Do not edit.
# Use project properties -> Build -> Qt -> Expert -> Custom Definitions.
TEMPLATE = app
DESTDIR = dist/BeagleBone-Debug/BeagleBone-Linux
TARGET = DMDSAR
VERSION = 1.0.0
CONFIG -= debug_and_release app_bundle lib_bundle
CONFIG += debug 
PKGCONFIG +=
QT = core gui widgets opengl
SOURCES += src/MadgwickTiltSensor.cpp src/AbstractIMU.cpp src/AbstractTiltSensor.cpp src/Emulated9DOFIMU.cpp src/IMUEmulatorControl.cpp src/LogIMU.cpp src/MPU9250.cpp src/MahonyTiltSensor.cpp src/MovingAverageTiltSensor.cpp src/PerformanceLap.cpp src/Protractor.cpp src/main.cpp
HEADERS += src/MadgwickTiltSensor.h src/AbstractIMU.h src/AbstractTiltSensor.h src/Emulated9DOFIMU.h src/IMUEmulatorControl.h src/LogIMU.h src/MPU9250.h src/MahonyTiltSensor.h src/MovingAverageTiltSensor.h src/PerformanceLap.h src/Protractor.h
FORMS += src/IMUEmulatorControl.ui
RESOURCES +=
TRANSLATIONS +=
OBJECTS_DIR = build/BeagleBone-Debug/BeagleBone-Linux
MOC_DIR = 
RCC_DIR = 
UI_DIR = 
QMAKE_CC = gcc
QMAKE_CXX = g++
DEFINES += 
INCLUDEPATH += 
LIBS += 
