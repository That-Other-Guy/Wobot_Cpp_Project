#pragma once

#include "BMP085.h"
#include "W_Types.hpp"
#include "APT_Types.h"



class APT_AirPressureTemp
{
private:
	W_COut_t TerminalSetting;
private:
	BMP085_Data_s Data;
	int i2c_device_handle;
	unsigned long conversion_time;	
private:
	APT_Data_s Var;
private:
	BMP085_Command_t Temp_Req;
	BMP085_Command_t Press_Req;
	APT_DataStatus_t StatusTemp;
	APT_DataStatus_t StatusPress;
public:
	APT_AirPressureTemp(BMP085_OSS_t _Oss, W_COut_t _Terminal);
	~APT_AirPressureTemp();
private:
	APT_Status_t Status;
	APT_State_t State;
	unsigned long timer_time;
private:
	void Initialize();
	void Request_UncompTemp();	
	void Request_UncompPress();	
	void ProcessResults();
public:
	int Update();
	long PrintTemperature();
	long PrintAirPressure();
};

