# This file is generated automatically. Do not edit.
# Use project properties -> Build -> Qt -> Expert -> Custom Definitions.
TEMPLATE = app
DESTDIR = dist/MSYS2-Debug/MSYS2-Windows
TARGET = DMDSAR
VERSION = 1.0.0
CONFIG -= debug_and_release app_bundle lib_bundle
CONFIG += debug 
PKGCONFIG +=
QT = core gui widgets opengl
SOURCES += src/AbstractIMU.cpp src/Emulated9DOFIMU.cpp src/IMUEmulatorControl.cpp src/LogIMU.cpp src/MPU9250.cpp src/PerformanceLap.cpp src/Protractor.cpp src/TiltSensor.cpp src/main.cpp
HEADERS += src/AbstractIMU.h src/Emulated9DOFIMU.h src/IMUEmulatorControl.h src/LogIMU.h src/MPU9250.h src/PerformanceLap.h src/Protractor.h src/TiltSensor.h
FORMS += src/IMUEmulatorControl.ui
RESOURCES +=
TRANSLATIONS +=
OBJECTS_DIR = build/MSYS2-Debug/MSYS2-Windows
MOC_DIR = 
RCC_DIR = 
UI_DIR = 
QMAKE_CC = gcc
QMAKE_CXX = g++
DEFINES += 
INCLUDEPATH += 
LIBS += 
