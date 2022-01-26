#pragma once
#include "ITG3200.h"


enum class Gyro_Status
{
	Idle,
	DataRequest,
	DataWait,
	Processing,
	NextTask,
};

enum class Gyro_Sm
{
	Initialize,
	RequestData,
	LoadData,
	ProcessResults,
};

typedef struct
{
	float X;
	float Y;
	float Z;
}GyroData;

class Gyroscope
{
private:
	ITG3200_Data Data;
	int i2c_device_handle;
private:
	Gyro_Status Status;
	Gyro_Sm Sm;
	unsigned long thyme;
private:
	GyroData G_Data;
	float fscaling_factor_x;
	float fscaling_factor_y;
	float fscaling_factor_z;
public:
	Gyroscope();
	~Gyroscope();
private:
	void Initialize();
	void ProcessResults();
public:
	GyroData Update();
};

