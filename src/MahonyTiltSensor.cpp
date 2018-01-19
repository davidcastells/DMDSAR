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
 * File:   MahonyTiltSensor.cpp
 * Author: dcr
 * 
 * Created on January 19, 2018, 4:08 PM
 */

#include "MahonyTiltSensor.h"

#include <stdio.h>
#include <math.h>

MahonyTiltSensor::MahonyTiltSensor(AbstractIMU* imu)
{
    m_pImu = imu;
    
    q0 = 1.0;
    q1 = 0.0;
    q2 = 0.0;
    q3 = 0.0;
    
    integralFBx = 0.0;
    integralFBy = 0.0;
    integralFBz = 0.0;
	
    // @todo arbitrary values, I do not know how to set them ?
    Ki = 0.0;
    Kp = 0.5;
    //anglesComputed = 0;
    
    m_lap.start();
}



MahonyTiltSensor::~MahonyTiltSensor()
{
}


void MahonyTiltSensor::getTilt(double* roll, double* pitch, double* yaw)
{
    double ax, ay, az;
    double gx, gy, gz;
    double mx, my, mz;
    m_pImu->getAcceleration(&ax, &ay, &az);
    m_pImu->getGyroscope(&gx, &gy, &gz);
    m_pImu->getMagnetometer(&mx, &my, &mz);
 
    if (verbosity)
        printf("a: %f %f %f g: %f %f %f m: %f %f %f\n", ax,ay,az,gx,gy,gz,mx,my,mz);
    m_lap.stop();
    mahonyUpdate(gx,gy,gz,ax,ay,az,mx,my,mz, m_lap.lap());
    if (verbosity)
        printf("q: %f %f %f %f\n", q0, q1, q2, q3);
    m_lap.start();
    
    toEulerAngles(roll, pitch, yaw);
    if (verbosity)
        printf("euler: %f %f %f\n", roll, pitch, yaw);
}
    


/**
 * 
 * @param gx gyroscope x reading in radians/s
 * @param gy gyroscope y reading in radians/s
 * @param gz gyroscope z reading in radians/s
 * @param ax accelerometer x reading in m/s^2
 * @param ay accelerometer y reading in m/s^2
 * @param az accelerometer z reading in m/s^2
 * @param ellapsedTime this is the elapsed time from last measure, in the original code was
 *  an inverse from sampling frequency, but I think that in some systems we can not
 *  be sure about losing samples
 */
void MahonyTiltSensor::mahonyUpdateIMU(double gx, double gy, double gz, double ax, double ay, double az, double elapsedTime)
{
    double recipNorm;
    double halfvx, halfvy, halfvz;
    double halfex, halfey, halfez;
    double qa, qb, qc;

    // Compute feedback only if accelerometer measurement valid
    // (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0) && (ay == 0.0) && (az == 0.0))) 
    {
        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Estimated direction of gravity, this is expressed as quaternions
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5 + q3 * q3;

        // Error is sum of cross product between estimated
        // and measured direction of gravity
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        // Compute and apply integral feedback if enabled
        if(Ki > 0.0) 
        {
            // integral error scaled by Ki
            integralFBx += 2.0 * Ki * halfex * elapsedTime;
            integralFBy += 2.0 * Ki * halfey * elapsedTime;
            integralFBz += 2.0 * Ki * halfez * elapsedTime;
            gx += integralFBx;	// apply integral feedback
            gy += integralFBy;
            gz += integralFBz;
        } 
        else 
        {
            integralFBx = 0.0f;	// prevent integral windup
            integralFBy = 0.0f;
            integralFBz = 0.0f;
        }

        // Apply proportional feedback
        gx += 2.0 * Kp * halfex;
        gy += 2.0 * Kp * halfey;
        gz += 2.0 * Kp * halfez;
    }

    // Integrate rate of change of quaternion
    gx *= (0.5 * elapsedTime);		// pre-multiply common factors
    gy *= (0.5 * elapsedTime);
    gz *= (0.5 * elapsedTime);
    qa = q0;
    qb = q1;
    qc = q2;
    q0 += (-qb * gx - qc * gy - q3 * gz);
    q1 += (qa * gx + qc * gz - q3 * gy);
    q2 += (qa * gy - qb * gz + q3 * gx);
    q3 += (qa * gz + qb * gy - qc * gx);

//    printf("muIMU q: %f %f %f %f\n", q0, q1, q2, q3);
        
    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;
    //anglesComputed = 0;
}

double MahonyTiltSensor::invSqrt(double x)
{
    double ret = 1.0 / sqrt(x);
    
//    printf("1/sq(%f) = %f\n", x, ret);
    return ret;
}

/**
 * This is the Mahony AHRS Filter  (Attitude Heading Reference System)
 * Inspired in 
 * https://github.com/adafruit/Adafruit_AHRS/blob/master/Mahony.cpp
 * 
 * I changed the use of float for double, used radians, and avoid performance optimizations
 *
 * @param gx gyroscope x reading in radians/s
 * @param gy gyroscope y reading in radians/s
 * @param gz gyroscope z reading in radians/s
 * @param ax accelerometer x reading in m/s^2
 * @param ay accelerometer y reading in m/s^2
 * @param az accelerometer z reading in m/s^2
 * @param mx
 * @param my
 * @param mz
 */
void MahonyTiltSensor::mahonyUpdate(double gx, double gy, double gz, double ax, double ay, double az, double mx, double my, double mz, double elapsedTime)
{
    double recipNorm;
    double q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    double hx, hy, bx, bz;
    double halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    double halfex, halfey, halfez;
    double qa, qb, qc;

    // Use IMU algorithm if magnetometer measurement invalid
    // (avoids NaN in magnetometer normalisation)
    if((mx == 0.0) && (my == 0.0) && (mz == 0.0)) 
    {
        mahonyUpdateIMU(gx, gy, gz, ax, ay, az, elapsedTime);
        return;
    }


	// Compute feedback only if accelerometer measurement valid
	// (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) 
        {
            // Normalise accelerometer measurement
            recipNorm = invSqrt(ax * ax + ay * ay + az * az);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Normalise magnetometer measurement
            recipNorm = invSqrt(mx * mx + my * my + mz * mz);
            mx *= recipNorm;
            my *= recipNorm;
            mz *= recipNorm;

            // Auxiliary variables to avoid repeated arithmetic
            q0q0 = q0 * q0;
            q0q1 = q0 * q1;
            q0q2 = q0 * q2;
            q0q3 = q0 * q3;
            q1q1 = q1 * q1;
            q1q2 = q1 * q2;
            q1q3 = q1 * q3;
            q2q2 = q2 * q2;
            q2q3 = q2 * q3;
            q3q3 = q3 * q3;

            // Reference direction of Earth's magnetic field
            hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
            hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
            bx = sqrt(hx * hx + hy * hy);
            bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

            // Estimated direction of gravity and magnetic field
            halfvx = q1q3 - q0q2;
            halfvy = q0q1 + q2q3;
            halfvz = q0q0 - 0.5f + q3q3;
            halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
            halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
            halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

            // Error is sum of cross product between estimated direction
            // and measured direction of field vectors
            halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
            halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
            halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

            // Compute and apply integral feedback if enabled
            if(Ki > 0.0) 
            {
                // integral error scaled by Ki
                integralFBx += 2.0 * Ki * halfex * elapsedTime;
                integralFBy += 2.0 * Ki * halfey * elapsedTime;
                integralFBz += 2.0 * Ki * halfez * elapsedTime;
                gx += integralFBx;	// apply integral feedback
                gy += integralFBy;
                gz += integralFBz;
            } 
            else 
            {
                    integralFBx = 0.0;	// prevent integral windup
                    integralFBy = 0.0;
                    integralFBz = 0.0;
            }

            // Apply proportional feedback
            gx += 2.0 * Kp * halfex;
            gy += 2.0 * Kp * halfey;
            gz += 2.0 * Kp * halfez;
	}

	// Integrate rate of change of quaternion
	gx *= (0.5f * elapsedTime);		// pre-multiply common factors
	gy *= (0.5f * elapsedTime);
	gz *= (0.5f * elapsedTime);
	qa = q0;
	qb = q1;
	qc = q2;
	q0 += (-qb * gx - qc * gy - q3 * gz);
	q1 += (qa * gx + qc * gz - q3 * gy);
	q2 += (qa * gy - qb * gz + q3 * gx);
	q3 += (qa * gz + qb * gy - qc * gx);

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
//	anglesComputed = 0;
}


void MahonyTiltSensor::toEulerAngles(double* roll, double* pitch, double* yaw)
{
    *roll = atan2(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
    *pitch = asin(-2.0f * (q1*q3 - q0*q2));
    *yaw = atan2(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);    
}

