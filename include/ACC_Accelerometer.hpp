#pragma once
#ifndef ACCELEROMETER_H_
#define ACCELEROMETER_H_

#include "ADXL345.h"
#include "ACC_Types.h"

class ACC_Accelerometer
{
private:
	ADXL345_Data_s Data;
	int fd_i2c_device;
private:
	ACC_Status_t Status;
	ACC_State_t State;
	unsigned long timer_time;
private:
	ACC_Data_s AccelData;
	float fscaling_factor;
public:
	ACC_Accelerometer(ADXL345_Rate_t _Rate, ADXL345_Gain_t _Gain);
	~ACC_Accelerometer();
private:
	void Initialize();
	void ProcessResults();
public:
	ACC_Data_s Update();
};

#endif // !ACCELEROMETER_H_

