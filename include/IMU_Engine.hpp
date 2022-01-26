#pragma once
#include "accelerometer.h"
#include "gyroscope.h"

class quarternion
{

};

class IMU
{
private:
	Accelerometer DeltaVee;
	Gyroscope Omega;
private:
	quarternion q;
public:

private:

public:
	int Update();
};

