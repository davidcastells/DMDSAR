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
 * File:   MPU9250.cpp
 * Author: dcr
 * 
 * Created on January 17, 2018, 3:55 PM
 * base it on 
 * https://raw.githubusercontent.com/Junle-Lu/BeagleUUV/master/IMU/MPU9250.cpp
 */

//============================================================================
// Name        : MPU9250.cpp
// Author      : Junle Lu
// Version     : 1.0
/*
 * MPU9250.cpp
 * Userland driver (dev-interface) with InvenSense MPU-9250 sensor
 * over the I2C bus
 *
 * Copyright Junle Lu, Watson School of Engineering and Applied Science, Binghamton University
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED ''AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL I
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
//============================================================================
#include <iostream>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>

#ifdef WIN32
    // Just to allow compilation
    #define I2C_SLAVE 0
    
    int ioctl(int , int , int){ return 0;}
#else
    #include <linux/i2c.h>
    #include <linux/i2c-dev.h>
    #include <sys/ioctl.h>
    #include <stropts.h>
#endif

#include <stdio.h>
#include <math.h>
#include "MPU9250.h"
#include <stdint.h>
#include <sys/stat.h>

using namespace std;
//i2c protocol for MPU9250
//read protocol:write to the file with the destination address with one byte, then multiple data can be read in sequence of the destination register.
//write protocol: write two bytes of data with first data to be the register address

//set all sensor scale values
unsigned char acc_scale = acc_scale_2g;
unsigned char gyro_scale = gyro_scale_250;
unsigned mag_scale = mag_scale_16bits;
unsigned char mag_mode = 0x02; //2 = 8hz, 6 = 100hz for continuous data read

MPU9250::MPU9250()
{
    get_sensor_resolution();
    mpu9250_detect();
    mpu9250_selftest();
    mpu9250_calibration();
    mpu9250_initialization();
    mag_initialization();
    mag_calibration();

    //debug_mode(); working in progress


    //read_pressure();
}

void MPU9250::mpu9250_detect()
{
	//open the i2c adapter
	cout<< "Detecting the MPU-9250 device..."<<endl;

	int file;

	//open the adapter file and check if the adapter exit in the system
	if ((file = open("/dev/i2c-2", O_RDWR))<0)
	{
		cout<<"The i2c bus does not exit!"<<endl;
	}

	//does not check if the device is attached to this address
	if (ioctl(file, I2C_SLAVE, 0x68)<0)
	{
		cout<<"I2C Slave address does not exit!"<<endl;
	}

	device_file = file;

	//default value is 0x71
	while(read_register(WHO_AM_I_MPU9250) != 0x71)
	{
		cout<<"The device failed...check connections!"<<endl;
		cout<<"Press Enter to continue..."<<endl;
		getchar();
	}

	cout<<">>The MPU-9250 connection is established!...."<<endl<<"\n";
}

void MPU9250::mpu9250_selftest()
{
	cout<<"MPU9250 self-testing..."<<endl;
	unsigned char full_scale = 0x00;
	double acceleration_temp[3] ={0};
	double gyroscope_temp[3] = {0};
	double acceleration_temp_test[3] ={0};
	double gyroscope_temp_test[3] = {0};
	double factory_trim[6];
	unsigned char self_test_code[6];

	write_register(SMPLRT_DIV, 0x00); 			//set gyro sample rate 1 kHz
	write_register(CONFIG, 0x02);	  			//set gyro sample rate 1 kHz and DLPF to 92 Hz
	write_register(GYRO_CONFIG, 1<<full_scale); //set full scale range for the gyro to 250 dps
	write_register(ACCEL_CONFIG2, 0x02);		//set accelerometer rate to 1 kHz and bandwidth to 92 Hz
	write_register(ACCEL_CONFIG, 1<<full_scale);//set full scale range for the accelerometer to 2g


	cout<<">>reading and calculating accelerometer and gyrocope average values with 200 samples"<<endl;
	for (int i = 0; i < 200; i++)
	{
		read_raw_mpu_data();
		acceleration_temp[0] += acceleration[0];
		acceleration_temp[0] /=2;
		acceleration_temp[1] += acceleration[1];
		acceleration_temp[1] /=2;
		acceleration_temp[2] += acceleration[2];
		acceleration_temp[2] /=2;

		gyroscope_temp[0] += gyroscope[0];
		gyroscope_temp[0] /= 2;
		gyroscope_temp[1] += gyroscope[1];
		gyroscope_temp[1] /= 2;
		gyroscope_temp[2] += gyroscope[2];
		gyroscope_temp[2] /= 2;

	}

	cout<<"The average values for acceleration and gyroscope:"<<endl;
	cout<<"Accelerometer x, y, z: "<< acceleration_temp[0]<<", "<<acceleration_temp[1]<<", "<<acceleration_temp[2]<<endl;
	cout<<"Gyroscope x, y, z: "<< gyroscope_temp[0]<<", "<<gyroscope_temp[1]<<", "<<gyroscope_temp[2]<<endl;
	cout<<endl;

	//configure the accelerometer for self-test
	write_register(ACCEL_CONFIG, 0xE0);		//enable self test on all three axes and set accelerometer range to +/- 2g
	write_register(GYRO_CONFIG, 0xE0);		//enable self test on all three axes and set gyro range to +/- 250
	usleep(25000);//delay to let device to stabilize


	//get average self test values
	for (int j = 0; j < 200; j++)
	{
		read_raw_mpu_data();
		acceleration_temp_test[0] += acceleration[0];
		acceleration_temp_test[0] /=2;
		acceleration_temp_test[1] += acceleration[1];
		acceleration_temp_test[1] /=2;
		acceleration_temp_test[2] += acceleration[2];
		acceleration_temp_test[2] /=2;

		gyroscope_temp_test[0] += gyroscope[0];
		gyroscope_temp_test[0] /= 2;
		gyroscope_temp_test[1] += gyroscope[1];
		gyroscope_temp_test[1] /= 2;
		gyroscope_temp_test[2] += gyroscope[2];
		gyroscope_temp_test[2] /= 2;

	}

	cout<<"The average self test values for acceleration and gyroscope:"<<endl;
	cout<<"Accelerometer x, y, z: "<< acceleration_temp_test[0]<<", "<<acceleration_temp_test[1]<<", "<<acceleration_temp_test[2]<<endl;
	cout<<"Gyroscope x, y, z: "<< gyroscope_temp_test[0]<<", "<<gyroscope_temp_test[1]<<", "<<gyroscope_temp_test[2]<<endl;

	//configure the gyro and accelerometer for normal operation
	write_register(ACCEL_CONFIG, 0x00);
	write_register(GYRO_CONFIG, 0x00);
	usleep(25000); //delay for device to stabilize

	//registers not in order, can't read all at once
	//retrieve accelerometer and gyroscope factory self-test code from USR_Reg
	self_test_code[0] = read_register(SELF_TEST_X_ACCEL);
	self_test_code[1] = read_register(SELF_TEST_Y_ACCEL);
	self_test_code[2] = read_register(SELF_TEST_Z_ACCEL);
	self_test_code[3] = read_register(SELF_TEST_X_GYRO);
	self_test_code[4] = read_register(SELF_TEST_Y_GYRO);
	self_test_code[5] = read_register(SELF_TEST_Z_GYRO);

	//retrieve factory self-test value from self-test code reads
	factory_trim[0] = (double)(2620/1<<full_scale)*(pow(1.01, ((double)self_test_code[0] - 1.0)));
	factory_trim[1] = (double)(2620/1<<full_scale)*(pow(1.01, ((double)self_test_code[1] - 1.0)));
	factory_trim[2] = (double)(2620/1<<full_scale)*(pow(1.01, ((double)self_test_code[2] - 1.0)));
	factory_trim[3] = (double)(2620/1<<full_scale)*(pow(1.01, ((double)self_test_code[3] - 1.0)));
	factory_trim[4] = (double)(2620/1<<full_scale)*(pow(1.01, ((double)self_test_code[4] - 1.0)));
	factory_trim[5] = (double)(2620/1<<full_scale)*(pow(1.01, ((double)self_test_code[5] - 1.0)));

	//+/- 14 is acceptable
	cout<<endl;
	cout<<"The self_test_result of accelerometer and gyroscope:"<<endl;
	for (int k = 0; k < 6; k++)
	{
		selftest_result[k] = ((double)(acceleration_temp_test[k] - acceleration_temp[k]))/factory_trim[k];
		cout<<selftest_result[k]<<endl;
	}

	int check = 0;

	for (int k = 0; k<6; k++)
	{
		if (selftest_result[k] > 1.14 || selftest_result[k] < 0.86)
		{
			cout<<">>self-testing failed!"<<endl;
			check = 0;
			break;
		}
		else
			check = 1;
	}

	if (check == 1)
		cout<<">>self-testing passed!"<<"\n\n"<<endl;

}

void MPU9250::mpu9250_calibration()
{
    cout<<"MPU-9250 calibration..."<<endl;
    unsigned char data_buffer[12] = {0};
    unsigned char gyrosensitivity = 131;
    unsigned char accelsensitivity = 16384;

    gyroBias[0] = 0;
    accBias[0] = 0;
    gyroBias[1] = 0;
    accBias[1] = 0;
    gyroBias[2] = 0;
    accBias[2] = 0;

    short fifo_count= 0;
    short packet_count = 0;

    //reset the device
    write_register(PWR_MGMT_1, 0x80);
    usleep(100000);

    //get stable time source; auto select clock source to be PPL gyrpscope reference if ready
    //else use the internal oscillator, bits 2:0 = 001
    write_register(PWR_MGMT_1, 0X01);
    write_register(PWR_MGMT_2, 0X00);
    usleep(200000);

    //configure device for bias calculation
    write_register(INT_ENABLE, 0X00);
    write_register(FIFO_EN, 0X00);
    write_register(PWR_MGMT_1, 0X00);
    write_register(I2C_MST_CTRL, 0X00);
    write_register(USER_CTRL, 0X00);
    write_register(USER_CTRL, 0X00);
    usleep(50000);

    //configure gyro and accelerometer for bias calculation
    write_register(CONFIG, 0X01);		//set low pass filter to 188 Hz
    write_register(SMPLRT_DIV, 0X00);	//set sample rate to 1 kHz
    write_register(GYRO_CONFIG, 0X00);	//set gyro full scale to 250 degrees per second
    write_register(ACCEL_CONFIG, 0X00);     //set accelerometer full scale to 2g,

    write_register(USER_CTRL, 0x40);//enable FIFO
    write_register(FIFO_EN, 0x78); //enable gyro and accelerometer sensors for FIFO
    usleep(40000); //accumulate 40 samples in 40 milliseconds = 480 bytes

    write_register(FIFO_EN, 0x00);
    data_buffer[0]= read_register(FIFO_COUNTH);
    data_buffer[1]= read_register(FIFO_COUNTL);
    fifo_count = bit_conversion(data_buffer[0], data_buffer[1]);
    packet_count = fifo_count/12; //how many sets of full gyro and accelerometer data for averaging

    cout<<">>the FIFO_count is "<<fifo_count<<endl;
    cout<<">>the packet_cout is "<<packet_count<<endl;

}

void MPU9250::mpu9250_initialization()
{
    cout<<"Now...initializing the device..."<<endl;
    write_register(PWR_MGMT_1, 0x00);//clear sleep mode, enable all sensors
    usleep(100000);//wait for register reset

    write_register(PWR_MGMT_1, 0x01);//auto select clock source to be PLL gyroscope reference if ready else
    usleep(200000);

    //configure gyro and thermometer
    //disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
    //minimum delay time for this setting is 5.9ms, which means sensor fusion update rates cannot be higher than 1/0.0059 = 170 Hz
    //DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
    //with the MPU9250, it is possible to get gyro sample rates of 32 kHz, 8kHz, or 1kHz
    write_register(CONFIG, 0x03);

    //set sample rate = gyroscope output rate/(1+SMPLR_DIV)
    write_register(SMPLRT_DIV, 0x04);//use a 200 Hz rate

    //set gyroscope full scale range
    //range selects FS_SEL and AFS_SEL are 0-3, so 2-bit values are left-shifted into positions 4:3
    unsigned char temp = read_register(GYRO_CONFIG);

    write_register(GYRO_CONFIG, temp & ~0x02);//clear Fchoice bits [1:0]
    write_register(GYRO_CONFIG, temp & ~0x18);//clear AFS bits [4:3]
    write_register(GYRO_CONFIG, temp | gyro_scale <<3); //set full scale range for the gyro

    //set accelerometer full-scale range configuration
    temp = read_register(ACCEL_CONFIG);

    write_register(ACCEL_CONFIG, temp & ~0x18); //clear AFS bits [4:3]
    write_register(ACCEL_CONFIG, temp | acc_scale <<3);//set full scale range for the accelerometer

    //set accelerometer sample rate configuration
    //it is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
    //accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
    temp = read_register(ACCEL_CONFIG2);
    write_register(ACCEL_CONFIG2, temp & ~0x0F); //clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
    write_register(ACCEL_CONFIG2, temp | 0x03); //set accelerometer rate for 1 kHz and bandwidth to 41 Hz

    //The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
    //but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

    //configure Interrupts and Bypass Enable
    //set interrupt pin active high, push-pull, hold interrupt pin level high until interrupt cleared,
    //clear on read of INT_STATUS, and eable I2C_BYPASS_EN

    cout<<"Disabling the device's I2c master feature..."<<endl;
    write_register(INT_PIN_CFG, 0x22);
    write_register(INT_ENABLE, 0x01); //enable data ready bit0 interrupt

    if (read_register(INT_PIN_CFG) != 0x22)
            cout<<">>Failed!"<<endl<<"\n";
    else
            cout<<">>Success!"<<endl<<"\n";

}



void MPU9250::mag_initialization()
{
    //open the i2c adapter
    cout<< "Detecting the magnetometer device...."<<endl;

    int file;
    unsigned char data_buffer[3];

    //open the adapter file and check if the adapter exit in the system
    if ((file = open("/dev/i2c-2", O_RDWR))<0)
    {
            cout<<"The i2c bus does not exit!"<<endl;
    }

    //does not check if the device is attached to this address
    if (ioctl(file, I2C_SLAVE, AK8963_ADDRESS)<0)
    {
            cout<<"I2C Slave address does not exit!"<<endl;
    }

    mag_file = file;

    //it should return 0x48
    while (read_mag_register(WHO_AM_I_AK8963) != 72 )
    {
            cout<<"The device failed...check connections!"<<endl;
            cout<<"Press Enter to continue..."<<endl;
            getchar();
    }

    cout<<">>The magnetometer connection is established!..."<<endl<<"\n";
    cout<<">>Now...initializing the device..."<<endl;

    //initialize and read sensitivity adjustment value
    cout<<"Powering down magnetometer..."<<endl;
    write_mag_register(AK8963_CNTL, 0x00);
    if (read_mag_register(AK8963_CNTL) == 0x00)
            cout<<">>Success!"<<endl;
    else
            cout<<">>Failed!"<<endl;
    usleep(10000);


    cout<<"Entering FUSE ROM access mode...reading calibration values"<<endl;
    write_mag_register(AK8963_CNTL, 0x0F);
    if (read_mag_register(AK8963_CNTL) == 0x0F)
            cout<<">>Success!"<<endl;
    else
            cout<<">>Failed!"<<endl;
    char the_address[1] = {AK8963_ASAX};//required for write function argument type
    write(mag_file, the_address, 1);
    read(mag_file,data_buffer, 3);
    mag_sensitivity_values[0]= (double)((data_buffer[0] - 128)/256. +1.);//equation from data sheet
    mag_sensitivity_values[1]= (double)((data_buffer[1] - 128)/256. +1.);
    mag_sensitivity_values[2]= (double)((data_buffer[2] - 128)/256. +1.);

    cout<<"the x value is "<<mag_sensitivity_values[0]<<endl;
    cout<<"the y value is "<<mag_sensitivity_values[1]<<endl;
    cout<<"the z value is "<<mag_sensitivity_values[2]<<"\n"<<endl;

    cout<<"Powering down magnetometer..."<<endl;
    write_mag_register(AK8963_CNTL, 0x00);
    if (read_mag_register(AK8963_CNTL) == 0x00)
            cout<<">>Success!"<<endl;
    else
            cout<<">>Failed!"<<endl;
    usleep(10000);


    cout<<"Setting magnetometer data scale and sample rate..."<<endl;
    write_mag_register(AK8963_CNTL, mag_scale<<4|mag_mode);
    if (read_mag_register(AK8963_CNTL) == 0x12)
            cout<<">>Success!.."<<endl;
    else
            cout<<">>Failed!..."<<endl;
    usleep(10000);

    cout<<"The mag_resolution is "<<mag_resolution<<" uT/unit"<<endl;
    cout<<"The acc_resolution is "<<acc_resolution<<" g/unit"<<endl;
    cout<<"The gyro_resolution is "<<gyro_resolution<<" dps/unit"<<endl;

}

void MPU9250::mag_calibration()
{
	cout<<"Magnetometer calibration: wave device in a figure 8 until it is done!"<<endl;
	usleep(1000000);
	cout<<"3!"<<endl;
	usleep(1000000);
	cout<<"2!"<<endl;
	usleep(1000000);
	cout<<"1!"<<endl;
	usleep(1000000);
	cout<<"Begin!"<<endl;
	usleep(1000000);

	int sample_count = 64;
	double temp_max[3] = {0,0,0};
	double temp_min[3] = {0,0,0};
	magBias[0] = 0;
	magBias[1] = 0;
	magBias[2] = 0;

	for (int i = 0; i <sample_count; i++)
	{
		read_raw_magnetometer();
		for (int j = 0; j<3; j++)
		{
			if (magnetometer[j] > temp_max[j])
				temp_max[j] = magnetometer[j];
			else if (magnetometer[j] < temp_min[j])
				temp_min[j] = magnetometer[j];
		}
		usleep(135000);//delay 135ms to make sure the next data is ready
	}

	cout<<"mag x min/max: "<< temp_min[0]<<"/"<<temp_max[0]<<endl;
	cout<<"mag x min/max: "<< temp_min[1]<<"/"<<temp_max[1]<<endl;
	cout<<"mag x min/max: "<< temp_min[2]<<"/"<<temp_max[2]<<endl;

	magBias[0] = (temp_min[0]+temp_max[0])/2*mag_resolution*mag_sensitivity_values[0];
	magBias[1] = (temp_min[1]+temp_max[1])/2*mag_resolution*mag_sensitivity_values[1];
	magBias[2] = (temp_min[2]+temp_max[2])/2*mag_resolution*mag_sensitivity_values[2];

	cout<<"The x mag bias value is "<<magBias[0]<<endl;
	cout<<"The y mag bias value is "<<magBias[1]<<endl;
	cout<<"The z mag bias value is "<<magBias[2]<<endl;

	cout<<">>Magnetometer calibration is done!"<<endl;

}

void MPU9250::get_sensor_resolution()
{
	switch(mag_scale)
	{
		case mag_scale_14bits:
			mag_resolution = 10.*4912./8190.;
			break;
		case mag_scale_16bits:
			mag_resolution = 10.*4912./32760.;
			break;
	}

	switch(acc_scale)
	{
		case acc_scale_2g:
			acc_resolution = 2./32768.;
			break;
		case acc_scale_4g:
			acc_resolution = 4./32768.;
			break;
		case acc_scale_8g:
			acc_resolution = 8./32768.;
			break;
		case acc_scale_16g:
			acc_resolution = 16./32768;
			break;
	}

	switch (gyro_scale)
	{
		case gyro_scale_250:
			gyro_resolution = 250./32768.;
			break;
		case gyro_scale_500:
			gyro_resolution = 500./32768.;
			break;
		case gyro_scale_1000:
			gyro_resolution = 1000./32768.;
			break;
	}
}

void MPU9250::read_raw_magnetometer()
{
	char read_address[1] = {AK8963_XOUT_L};
	unsigned char data_buffer[7];

	if(read_mag_register(AK8963_ST1) & 0x01)//check if the data is ready
	{
            //read x,y,z value with data status register
            write(mag_file, read_address, 1);
            read(mag_file, data_buffer, 7);
	}

	if (!(data_buffer[6] & 0x08))
	{
            //convert raw data into signed 16-bit value in little endian format
            magnetometer[0] = bit_conversion(data_buffer[1], data_buffer[0]);
            magnetometer[1] = bit_conversion(data_buffer[3], data_buffer[2]);
            magnetometer[2] = bit_conversion(data_buffer[5], data_buffer[4]);
            //cout<< "The magnetometer x is "<<magnetometer[0]<<endl;
            //cout<< "The magnetometer y is "<<magnetometer[1]<<endl;
            //cout<< "The magnetometer z is "<<magnetometer[2]<<endl;
	}

}


/**
 * We convert milligause to gause
 * @param mx
 * @param my
 * @param mz
 */
void MPU9250::getMagnetometer(double* mx, double *my, double* mz) 
{
    read_raw_magnetometer();
    *mx = magnetometer[0]/ 1000.0;  
    *my = magnetometer[1]/ 1000.0;
    *mz = magnetometer[2]/ 1000.0;
    
    
}

/**
 * @deprecated This should be part of the TiltSensor, not the IMU itself
 * @param data
 * @param yaw
 */
void MPU9250::get_actual_mag_data(double data[3], double* yaw)
{
	read_raw_magnetometer();
	/*cout<<"The mag resolution is "<<mag_resolution<<endl;
	cout<<"The x sensitivity value:"<<mag_sensitivity_values[0]<<endl;
	cout<<"The y sensitivity value:"<<mag_sensitivity_values[1]<<endl;
	cout<<"The z sensitivity value:"<<mag_sensitivity_values[2]<<endl;
*/
	magnetometer[0] = (double)((magnetometer[0]*mag_resolution*mag_sensitivity_values[0]) - magBias[0]);
	magnetometer[1] = (double)((magnetometer[1]*mag_resolution*mag_sensitivity_values[1]) - magBias[1]);
	magnetometer[2] = (double)((magnetometer[2]*mag_resolution*mag_sensitivity_values[2]) - magBias[2]);

	/*cout<< "The magnetometer x is "<<magnetometer[0]<<endl;
	cout<< "The magnetometer y is "<<magnetometer[1]<<endl;
	cout<< "The magnetometer z is "<<magnetometer[2]<<endl;
*/
	data[0] = magnetometer[0];
	data[1] = magnetometer[1];
	data[2] = magnetometer[2];
	*yaw  = atan2(magnetometer[1],magnetometer[0])*180/M_PI;
	if (*yaw < 0)
		*yaw += 360;

}

void MPU9250::read_raw_mpu_data()//read all data at once to reduce communication overhead
{

	//sensor register address begins 0x3b and high to low bytes
	unsigned char data_buffer[14];
	char initial_address[1] = {ACCEL_XOUT_H};


	write(device_file, initial_address, 1);
	read(device_file, data_buffer, 14);

	acceleration[0] = bit_conversion(data_buffer[0], data_buffer[1]);
	acceleration[1] = bit_conversion(data_buffer[2], data_buffer[3]);
	acceleration[2] = bit_conversion(data_buffer[4], data_buffer[5]);
	temperature = bit_conversion(data_buffer[6], data_buffer[7]);
	gyroscope[0] = bit_conversion(data_buffer[8], data_buffer[9]);
	gyroscope[1] = bit_conversion(data_buffer[10], data_buffer[11]);
	gyroscope[2] = bit_conversion(data_buffer[12], data_buffer[13]);

}

/**
 * We convert mg to m/s^2
 * @param ax
 * @param ay
 * @param az
 */
void MPU9250::getAcceleration(double* ax, double *ay, double* az)
{
    read_raw_mpu_data();
    *ax = acceleration[0] / 1000.0; 
    *ay = acceleration[1] / 1000.0;
    *az = acceleration[2] / 1000.0;
}
    
/**
 * We convert degress to radians
 * @param gx
 * @param gy
 * @param gz
 */
void MPU9250::getGyroscope(double* gx, double *gy, double* gz)
{   
    static const double PI=3.141592;
    
    // @todo We assume NOW that getGyroscope will be always called 
    // after getAcceleration, so we avoid calling read_raw_mpu_data
    *gx = gyroscope[0] * PI / 180.0;
    *gy = gyroscope[1] * PI / 180.0;
    *gz = gyroscope[2] * PI / 180.0;
}

void MPU9250::get_actual_mpu_data(double data[7], double* pitch, double* roll)
{
	if(read_register(INT_STATUS) & 0X01)//Check if data is ready
	{
		read_raw_mpu_data();
		acceleration[0] = (double) acceleration[0] * acc_resolution - accBias[0];
		acceleration[1] = (double) acceleration[1] * acc_resolution - accBias[1];
		acceleration[2] = (double) acceleration[2] * acc_resolution - accBias[2];
		gyroscope[0]= (double)gyroscope[0] * gyro_resolution - gyroBias[0];
		gyroscope[1]= (double)gyroscope[1] * gyro_resolution - gyroBias[1];
		gyroscope[2]= (double)gyroscope[2] * gyro_resolution - gyroBias[2];
		temperature = ((double)temperature)/333.87 +21.0;
		data[0] = acceleration [0];
		data[1] = acceleration [1];
		data[2] = acceleration [2];
		data[3] = gyroscope [0];
		data[4] = gyroscope [1];
		data[5] = gyroscope [2];
		data[6] = temperature;

		*pitch = (180 * atan(acceleration[0]/sqrt(pow(acceleration[1],2) + pow(acceleration[2],2)))/M_PI);
		*roll = (180 * atan(acceleration[1]/sqrt(pow(acceleration[0],2) + pow(acceleration[2],2)))/M_PI);

	}


	/*
	cout<< "The acceleration x is "<<acceleration[0]<<endl;
	cout<< "The acceleration y is "<<acceleration[1]<<endl;
	cout<< "The acceleration z is "<<acceleration[2]<<endl;
	cout<< "The temperature is "<<temperature<<endl;
	cout<< "The gyroscope x is "<<gyroscope[0]<<endl;
	cout<< "The gyroscope y is "<<gyroscope[1]<<endl;
	cout<< "The gyroscope z is "<<gyroscope[2]<<endl;
	cout<<endl;*/
}


/* Pressure Sensor Currently not working
void MPU9250::read_pressure()
{
	//open the i2c adapter
	cout<< "deviceRead() Test"<<endl;

	int file;

	//open the adapter file and check if the adapter exit in the system
	if ((file = open("/dev/i2c-1", O_RDWR))<0)
	{
		cout<<"The i2c bus does not exit!"<<endl;
	}

	//does not check if the device is attached to this address
	if (ioctl(file, I2C_SLAVE, 0x77)<0)
	{
		cout<<"I2C Slave address does not exit!"<<endl;
	}

	device_file = file;

	cout<<read_register(0xed)<<endl;

	//check if the device is successfully attach to the beagleBone
	while(read_register(WHO_AM_I_MPU9250) == 117)
	{
		cout<<"The device failed...check connections!"<<endl;
		cout<<"Press Enter to continue..."<<endl;
		getchar();
	}*/

int MPU9250::read_register(char register_address)
{
	char address[1] = {register_address};
	write(device_file, address, 1);
	read(device_file, address, 1);

	return (int)address[0];
}

//the data pointer shall point to char array contains register address and data
void MPU9250::write_register(char register_address, char data)
{
	//int temp = read_register(register_address);
	/*
	cout<<endl;
	cout<<"The current value in register "<<(int)register_address<<'(';
	display_binary((int)register_address);
	cout<<") is ";
	display_binary(temp);
	cout<<"."<<endl;*/

	char data_buffer[2] = {register_address, data};
	write(device_file, data_buffer, 2);
    /*
	temp = read_register(register_address);
	cout<<"The modified value in register "<<(int)register_address<<'(';
	display_binary((int)register_address);
	cout<<") is ";
	display_binary(temp);
	cout<<"."<<endl;*/
}

//read and write registers in magnetometer
int MPU9250::read_mag_register(char register_address)
{
	char address[1] = {register_address};
	write(mag_file, address, 1);
	read(mag_file, address, 1);

	return (int)address[0];
}

//the data pointer shall point to char array contains register address and data
void MPU9250::write_mag_register(char register_address, char data)
{
	/*int temp = read_mag_register(register_address);
	cout<<"The current value in register "<<(int)register_address<<'(';
	display_binary((int)register_address);
	cout<<") is ";
	display_binary(temp);
	cout<<"."<<endll*/

	char data_buffer[2] = {register_address, data};
	write(mag_file, data_buffer, 2);

	/*temp = read_mag_register(register_address);
	cout<<"The modified value in register "<<(int)register_address<<'(';
	display_binary((int)register_address);
	cout<<") is ";
	display_binary(temp);
	cout<<"."<<endl;*/
}
//combine two unsigned 8-bits to a signed 16-bit integer
short MPU9250::bit_conversion(unsigned char msb, unsigned char lsb)
{
	short temp = msb;
	temp = (temp<<8)|(lsb&0xff);
	return temp;
}

void MPU9250::display_binary(int temp)
{
	while (temp)
	{
		if (temp & 1)
			cout<<'1';
		else
			cout<<'0';
		temp >>= 1;
	}

	if (temp == 0)
		cout<<'0';
}

void MPU9250::debug_mode()
{
	int user_input;
	cout<<"Entering the debug mode..."<<endl;

	switch (user_input = debug_mode_menu())
	{
            case '1':
                    MPU9250_debug();
            case '2':
                    mag_debug();
            default:
                    break;
	}
}

char MPU9250::debug_mode_menu()
{
	char user_input;
	cout<<"Which device would you want to access?"<<"\n"<<"1->MPU9250"<<endl<<"2->magnetometer"<<endl;
	while ((user_input != 'q')&&(user_input != 'Q'))
	{

		user_input = getchar();
		if ((user_input == '1') || (user_input == '2'))
			break;

		if((user_input != '\n')&&(user_input != 'q')&&(user_input != 'Q')&&(user_input != '1') && (user_input != '2'))
			cout<<"Try again..."<<endl;
	}

	return user_input;
}

void MPU9250::MPU9250_debug()
{
	char user_input;
	int temp;

	while(1)
	{
		cout<<"Enter the register value in hex..."<<endl;
		user_input = getchar();

		while (user_input != '\n')
		{
			temp = read_register(user_input);

			cout<<endl;
			cout<<"The current value in register "<<(int)user_input<<'(';
			display_binary((int)user_input);
			cout<<") is ";
			display_binary(temp);
			cout<<"."<<endl;

		}

	}
}

void MPU9250::mag_debug()
{
}

MPU9250::~MPU9250()
{
	// TODO destructor
}
