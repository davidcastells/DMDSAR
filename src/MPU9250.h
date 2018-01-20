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
 * File:   MPU9250.h
 * Author: dcr
 *
 * Created on January 17, 2018, 3:55 PM
 * based on
 * https://raw.githubusercontent.com/Junle-Lu/BeagleUUV/master/IMU/MPU9250.h
 * 
 * check the MPU9250 datasheet docs
 * https://www.invensense.com/wp-content/uploads/2015/02/PS-MPU-9250A-01-v1.1.pdf
 * https://cdn.sparkfun.com/assets/learn_tutorials/5/5/0/MPU-9250-Register-Map.pdf
 */

#ifndef MPU9250_H
#define MPU9250_H

#include "AbstractIMU.h"


#define AK8963_ADDRESS   0x0C
#define WHO_AM_I_AK8963  0x00 // should return 0x48
#define INFO             0x01
#define AK8963_ST1       0x02  // data ready status bit 0
#define AK8963_XOUT_L	 0x03  // data
#define AK8963_XOUT_H	 0x04
#define AK8963_YOUT_L	 0x05
#define AK8963_YOUT_H	 0x06
#define AK8963_ZOUT_L	 0x07
#define AK8963_ZOUT_H	 0x08
#define AK8963_ST2       0x09  // Data overflow bit 3 and data read error status bit 2
#define AK8963_CNTL      0x0A  // Power down (0000), single-measurement (0001), self-test (1000) and Fuse ROM (1111) modes on bits 3:0
#define AK8963_ASTC      0x0C  // Self test control
#define AK8963_I2CDIS    0x0F  // I2C disable
#define AK8963_ASAX      0x10  // Fuse ROM x-axis sensitivity adjustment value
#define AK8963_ASAY      0x11  // Fuse ROM y-axis sensitivity adjustment value
#define AK8963_ASAZ      0x12  // Fuse ROM z-axis sensitivity adjustment value

#define SELF_TEST_X_GYRO 0x00
#define SELF_TEST_Y_GYRO 0x01
#define SELF_TEST_Z_GYRO 0x02

/*#define X_FINE_GAIN      0x03 // [7:0] fine gain
#define Y_FINE_GAIN      0x04
#define Z_FINE_GAIN      0x05
#define XA_OFFSET_H      0x06 // User-defined trim values for accelerometer
#define XA_OFFSET_L_TC   0x07
#define YA_OFFSET_H      0x08
#define YA_OFFSET_L_TC   0x09
#define ZA_OFFSET_H      0x0A
#define ZA_OFFSET_L_TC   0x0B */

#define SELF_TEST_X_ACCEL 0x0D
#define SELF_TEST_Y_ACCEL 0x0E
#define SELF_TEST_Z_ACCEL 0x0F

#define SELF_TEST_A      0x10

#define XG_OFFSET_H      0x13  // User-defined trim values for gyroscope
#define XG_OFFSET_L      0x14
#define YG_OFFSET_H      0x15
#define YG_OFFSET_L      0x16
#define ZG_OFFSET_H      0x17
#define ZG_OFFSET_L      0x18
#define SMPLRT_DIV       0x19
#define CONFIG           0x1A
#define GYRO_CONFIG      0x1B
#define ACCEL_CONFIG     0x1C
#define ACCEL_CONFIG2    0x1D
#define LP_ACCEL_ODR     0x1E
#define WOM_THR          0x1F

#define MOT_DUR          0x20  // Duration counter threshold for motion interrupt generation, 1 kHz rate, LSB = 1 ms
#define ZMOT_THR         0x21  // Zero-motion detection threshold bits [7:0]
#define ZRMOT_DUR        0x22  // Duration counter threshold for zero motion interrupt generation, 16 Hz rate, LSB = 64 ms

#define FIFO_EN          0x23
#define I2C_MST_CTRL     0x24
#define I2C_SLV0_ADDR    0x25
#define I2C_SLV0_REG     0x26
#define I2C_SLV0_CTRL    0x27
#define I2C_SLV1_ADDR    0x28
#define I2C_SLV1_REG     0x29
#define I2C_SLV1_CTRL    0x2A
#define I2C_SLV2_ADDR    0x2B
#define I2C_SLV2_REG     0x2C
#define I2C_SLV2_CTRL    0x2D
#define I2C_SLV3_ADDR    0x2E
#define I2C_SLV3_REG     0x2F
#define I2C_SLV3_CTRL    0x30
#define I2C_SLV4_ADDR    0x31
#define I2C_SLV4_REG     0x32
#define I2C_SLV4_DO      0x33
#define I2C_SLV4_CTRL    0x34
#define I2C_SLV4_DI      0x35
#define I2C_MST_STATUS   0x36
#define INT_PIN_CFG      0x37
#define INT_ENABLE       0x38
#define DMP_INT_STATUS   0x39  // Check DMP interrupt
#define INT_STATUS       0x3A
#define ACCEL_XOUT_H     0x3B
#define ACCEL_XOUT_L     0x3C
#define ACCEL_YOUT_H     0x3D
#define ACCEL_YOUT_L     0x3E
#define ACCEL_ZOUT_H     0x3F
#define ACCEL_ZOUT_L     0x40
#define TEMP_OUT_H       0x41
#define TEMP_OUT_L       0x42
#define GYRO_XOUT_H      0x43
#define GYRO_XOUT_L      0x44
#define GYRO_YOUT_H      0x45
#define GYRO_YOUT_L      0x46
#define GYRO_ZOUT_H      0x47
#define GYRO_ZOUT_L      0x48
#define EXT_SENS_DATA_00 0x49
#define EXT_SENS_DATA_01 0x4A
#define EXT_SENS_DATA_02 0x4B
#define EXT_SENS_DATA_03 0x4C
#define EXT_SENS_DATA_04 0x4D
#define EXT_SENS_DATA_05 0x4E
#define EXT_SENS_DATA_06 0x4F
#define EXT_SENS_DATA_07 0x50
#define EXT_SENS_DATA_08 0x51
#define EXT_SENS_DATA_09 0x52
#define EXT_SENS_DATA_10 0x53
#define EXT_SENS_DATA_11 0x54
#define EXT_SENS_DATA_12 0x55
#define EXT_SENS_DATA_13 0x56
#define EXT_SENS_DATA_14 0x57
#define EXT_SENS_DATA_15 0x58
#define EXT_SENS_DATA_16 0x59
#define EXT_SENS_DATA_17 0x5A
#define EXT_SENS_DATA_18 0x5B
#define EXT_SENS_DATA_19 0x5C
#define EXT_SENS_DATA_20 0x5D
#define EXT_SENS_DATA_21 0x5E
#define EXT_SENS_DATA_22 0x5F
#define EXT_SENS_DATA_23 0x60
#define MOT_DETECT_STATUS 0x61
#define I2C_SLV0_DO      0x63
#define I2C_SLV1_DO      0x64
#define I2C_SLV2_DO      0x65
#define I2C_SLV3_DO      0x66
#define I2C_MST_DELAY_CTRL 0x67
#define SIGNAL_PATH_RESET  0x68
#define MOT_DETECT_CTRL  0x69
#define USER_CTRL        0x6A  // Bit 7 enable DMP, bit 3 reset DMP
#define PWR_MGMT_1       0x6B // Device defaults to the SLEEP mode
#define PWR_MGMT_2       0x6C
#define DMP_BANK         0x6D  // Activates a specific bank in the DMP
#define DMP_RW_PNT       0x6E  // Set read/write pointer to a specific start address in specified DMP bank
#define DMP_REG          0x6F  // Register in DMP from which to read or to which to write
#define DMP_REG_1        0x70
#define DMP_REG_2        0x71
#define FIFO_COUNTH      0x72
#define FIFO_COUNTL      0x73
#define FIFO_R_W         0x74
#define WHO_AM_I_MPU9250 0x75 // Should return 0x71
#define XA_OFFSET_H      0x77
#define XA_OFFSET_L

//the data is 20x78
#define YA_OFFSET_H      0x7A
#define YA_OFFSET_L      0x7B
#define ZA_OFFSET_H      0x7D
#define ZA_OFFSET_L      0x7E


// PWR_MGMT_1 register bits
#define PWR_MGMT_1_HRESET           0x80
#define PWR_MGMT_1_SLEEP            0x40
#define PWR_MGMT_1_CYCLE            0x20
#define PWR_MGMT_1_GYRO_STANDBY     0x10
#define PWR_MGMT_1_PDTAT            0x08
#define PWR_MGMT_1_CLKSEL_MASK      (0x04|0x02|0x01)
#define PWR_MGMT_1_CLKSEL_INTERNAL_20MHZ      0
#define PWR_MGMT_1_CLKSEL_AUTOSELECT1         1
#define PWR_MGMT_1_CLKSEL_AUTOSELECT2         2
#define PWR_MGMT_1_CLKSEL_AUTOSELECT3         3
#define PWR_MGMT_1_CLKSEL_AUTOSELECT4         4
#define PWR_MGMT_1_CLKSEL_AUTOSELECT5         5
#define PWR_MGMT_1_CLKSEL_AUTOSELECT6         6
#define PWR_MGMT_1_CLKSEL_STOP                7

// PWR_MGMT_2 register bits
//#define PWR_MGMT_2_RESERVED         0x80
//#define PWR_MGMT_2_RESERVED         0x40
#define PWR_MGMT_2_DIS_XA           0x20
#define PWR_MGMT_2_DIS_YA           0x10
#define PWR_MGMT_2_DIS_ZA           0x08
#define PWR_MGMT_2_DIS_XG           0x04
#define PWR_MGMT_2_DIS_YG           0x02
#define PWR_MGMT_2_DIS_ZG           0x01
#define PWR_MGMT_2_ENABLE_ALL       0x00           


// GYRO_CONFIG register bits
#define GYRO_CONFIG_XSELFTEST       0x80
#define GYRO_CONFIG_YSELFTEST       0x40
#define GYRO_CONFIG_ZSELFTEST       0x20
#define GYRO_CONFIG_FULL_SCALE_1    0x10
#define GYRO_CONFIG_FULL_SCALE_0    0x08
//#define GYRO_CONFIG_RESERVED        0x04
#define GYRO_CONFIG_BYPASS_LPF1     0x02
#define GYRO_CONFIG_BYPASS_LPF0     0x01
#define GYRO_CONFIG_FS_250_DPS      0x00
#define GYRO_CONFIG_FS_500_DPS      GYRO_CONFIG_FULL_SCALE_0
#define GYRO_CONFIG_FS_1000_DPS     GYRO_CONFIG_FULL_SCALE_1
#define GYRO_CONFIG_FS_2000_DPS     (GYRO_CONFIG_FULL_SCALE_1|GYRO_CONFIG_FULL_SCALE_0)

#define ACCELEROMETER_FULL_SCALE    32768.0
#define GYROSCOPE_FULL_SCALE        32768.0
#define MAGNETOMETER_FULL_SCALE     32768.0
#define MAGNETOMETER_SCALE_14        8190.0

//list of scales for accelerometer, gyroscope, and magnetometer
enum acc_Scale 
{
    acc_scale_2g = 0,
    acc_scale_4g,
    acc_scale_8g,
    acc_scale_16g
};

// @deprecated use GYRO_CONFIG_FS_... constants
//enum gyro_Scale
//{
//    gyro_scale_250 = 0,
//    gyro_scale_500,
//    gyro_scale_1000,
//    gyro_scale_2000
//};

enum mag_Scale
{
	mag_scale_14bits = 0,
	mag_scale_16bits,
};



/**
 * MPU9250 IMU connected by I2C to the host
 * Derived from AbstractIMU 
 */
class MPU9250 : public AbstractIMU
{
// overides from AbstractIMU
public:
    void getAcceleration(double* ax, double *ay, double* az);
    void getGyroscope(double* gx, double *gy, double* gz);
    void getMagnetometer(double* mx, double *my, double* mz); 
    
private:

    int device_file;
    int mag_file;
    double mag_sensitivity_values[3];
    double mag_resolution;  //milliGause
    double acc_resolution;  // g
    double gyro_resolution; // degrees per second
    double gyroBias[3];
    double accBias[3];
    double magBias[3];
    double selftest_result[6];

    //store the value in x,y,z order
    double acceleration[3];
    double gyroscope[3];
    double magnetometer[3];
    double temperature;
    double pressure;
    //double pitch, roll, yaw;

    // Constructor / Destructor
public:
    MPU9250();
    virtual ~MPU9250();

private:
    void detect();  //detect the presence MPU9250 IC, and begin i2c protocol
    
public:
    //device configurations
    void mpu9250_initialization();//set registers' value as register value will reset when the device is powered up
    void mpu9250_selftest();
    void mpu9250_calibration();//perform sensor test. and calibrate accordingl

    void mag_initialization();//detect and set registers' value in magnetometer
    void mag_calibration();

    void get_sensor_resolution();//only need to do it once with the pre-defined values

    //raw data reading
    void read_raw_magnetometer();
    void read_raw_mpu_data();
    
    //void read_pressure();

    //register access and conversion
    short bit_conversion(unsigned char msb, unsigned char lsb);
    void write_register(char register_address, char data);
    int read_register(char register_address);
    void write_mag_register(char register_address, char data);
    int read_mag_register(char register_address);

    

};

#endif /* MPU9250_H */

