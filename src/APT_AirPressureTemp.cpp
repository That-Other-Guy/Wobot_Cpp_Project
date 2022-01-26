#include "APT_AirPressureTemp.h"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <cstring>

APT_AirPressureTemp::APT_AirPressureTemp(BMP085_OSS_t _Oss, W_COut_t TerminalSetting)
{
	State = APT_State_t::Initialize;
	memset(&Var, 0, sizeof(APT_Data_s));
	
	Var.OSS = (short)_Oss;

	Temp_Req = BMP085_Command_t::RequestTemp;

	switch (_Oss)
	{
	case BMP085_OSS_t::UltraLowPower:
		Press_Req = BMP085_Command_t::RequestPress_ULP;
		conversion_time = 5;
		break;
	case BMP085_OSS_t::Standard:
		Press_Req = BMP085_Command_t::RequestPress_STD;
		conversion_time = 8;
		break;
	case BMP085_OSS_t::HighResolution:
		Press_Req = BMP085_Command_t::RequestPress_HR;
		conversion_time = 14;
		break;
	case BMP085_OSS_t::UltraHighResolution:
		Press_Req = BMP085_Command_t::RequestPress_UHR;
		conversion_time = 26;
		break;
	default:
		break;
	}
}

APT_AirPressureTemp::~APT_AirPressureTemp()
{
}

void APT_AirPressureTemp::Initialize()
{
	i2c_device_handle = wiringPiI2CSetup((int)BMP085_DevAddr_t::Default);

	Data.BMP085_AC1_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC1_MSB);
	Data.BMP085_AC1_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC1_LSB);
	Data.BMP085_AC2_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC2_MSB);
	Data.BMP085_AC2_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC2_LSB);
	Data.BMP085_AC3_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC3_MSB);
	Data.BMP085_AC3_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC3_LSB);
	Data.BMP085_AC4_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC4_MSB);
	Data.BMP085_AC4_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC4_LSB);
	Data.BMP085_AC5_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC5_MSB);
	Data.BMP085_AC5_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC5_LSB);
	Data.BMP085_AC6_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC6_MSB);
	Data.BMP085_AC6_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::AC6_LSB);

	Data.BMP085_B1_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::B1_MSB);
	Data.BMP085_B1_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::B1_LSB);
	Data.BMP085_B2_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::B2_MSB);
	Data.BMP085_B2_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::B2_LSB);

	Data.BMP085_MB_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::MB_MSB);
	Data.BMP085_MB_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::MB_LSB);
	Data.BMP085_MC_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::MC_MSB);
	Data.BMP085_MC_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::MC_LSB);
	Data.BMP085_MD_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::MD_MSB);
	Data.BMP085_MD_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::MD_LSB);

	Var.AC1 = (short)((Data.BMP085_AC1_MSB << 8) | Data.BMP085_AC1_LSB);
	Var.AC2 = (short)((Data.BMP085_AC2_MSB << 8) | Data.BMP085_AC2_LSB);
	Var.AC3 = (short)((Data.BMP085_AC3_MSB << 8) | Data.BMP085_AC3_LSB);
	Var.AC4 = (unsigned short)((Data.BMP085_AC4_MSB << 8) | Data.BMP085_AC4_LSB);
	Var.AC5 = (unsigned short)((Data.BMP085_AC5_MSB << 8) | Data.BMP085_AC5_LSB);
	Var.AC6 = (unsigned short)((Data.BMP085_AC6_MSB << 8) | Data.BMP085_AC6_LSB);

	Var.B1 = (short)((Data.BMP085_B1_MSB << 8) | Data.BMP085_B1_LSB);
	Var.B2 = (short)((Data.BMP085_B2_MSB << 8) | Data.BMP085_B2_LSB);
				 						 
	Var.MB = (short)((Data.BMP085_MB_MSB << 8) | Data.BMP085_MB_LSB);
	Var.MC = (short)((Data.BMP085_MC_MSB << 8) | Data.BMP085_MC_LSB);
	Var.MD = (short)((Data.BMP085_MD_MSB << 8) | Data.BMP085_MD_LSB);

	Status = APT_Status_t::Idle;
}

void APT_AirPressureTemp::Request_UncompTemp()
{
	switch (Status)
	{
	case APT_Status_t::Idle:
		Status = APT_Status_t::DataRequest;
		break;
	case APT_Status_t::DataRequest:
		wiringPiI2CWriteReg8(i2c_device_handle, (int)BMP085_RegAddr_t::COMMAND, (int)Temp_Req);
		timer_time = millis();
		Status = APT_Status_t::DataWait;
		break;
	case APT_Status_t::DataWait:
		if ((millis() - timer_time) < conversion_time) break; 
		Data.BMP085_DATA_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::DATA_MSB);
		Data.BMP085_DATA_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::DATA_LSB);
		Var.UT = (long)(Data.BMP085_DATA_MSB << 8 | Data.BMP085_DATA_LSB);
		Status = APT_Status_t::NextTask;
		break;
	case APT_Status_t::Processing:
		Status = APT_Status_t::Idle;
		break;
	default:
		Status = APT_Status_t::Idle;
		break;
	}
	return;
}

void APT_AirPressureTemp::Request_UncompPress()
{
	switch (Status)
	{
	case APT_Status_t::Idle:
		Status = APT_Status_t::DataRequest;
		break;
	case APT_Status_t::DataRequest:
		{			
			wiringPiI2CWriteReg8(i2c_device_handle, (int)BMP085_RegAddr_t::COMMAND, (int)((int)Press_Req + ((uint8_t)Var.OSS<<6)));
			timer_time = millis();
			Status = APT_Status_t::DataWait;
		}
		break;
	case APT_Status_t::DataWait:
		if ((millis() - timer_time) < conversion_time) break;
		Data.BMP085_DATA_MSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::DATA_MSB);
		Data.BMP085_DATA_LSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::DATA_LSB);
		Data.BMP085_DATA_XLSB = (uint8_t)wiringPiI2CReadReg8(i2c_device_handle, (int)BMP085_RegAddr_t::DATA_XLSB);
		Var.UP = (long)( ((Data.BMP085_DATA_MSB << 16) | (Data.BMP085_DATA_LSB << 8) | (Data.BMP085_DATA_XLSB)) >> (8 - Var.OSS) );
		Status = APT_Status_t::NextTask;
		break;
	case APT_Status_t::Processing:
		Status = APT_Status_t::Idle;
		break;
	default:
		Status = APT_Status_t::Idle;
		break;
	}
	return;
}

void APT_AirPressureTemp::ProcessResults()
{
	Status = APT_Status_t::Processing;

	Var.X1 = ((Var.UT - Var.AC6) * (Var.AC5)) / (2 << 15);
	Var.X2 = (Var.MC * (2 << 11)) / (Var.X1 + Var.MD);
	Var.B5 = (Var.X1 + Var.X2);
	Var.T = (Var.B5 + 8) / (2 << 4);

	Var.B6 = Var.B5 - 4000;
	Var.X1 = (Var.B2 * ((Var.B6 * Var.B6) / (2 << 12))) / (2 << 11);
	Var.X3 = Var.X1 + Var.X2;
	Var.B3 = (((Var.AC1 * 4) + Var.X3) << (Var.OSS + 2)) / 4;
	Var.X1 = (Var.AC3 * Var.B6) / (2 << 13);
	Var.X2 = (Var.B1 * ((Var.B6 * Var.B6) / (2 << 12))) / (2 << 16);
	Var.X3 = ((Var.X1 + Var.X2) + 2) / (2 << 2);
	Var.B4 = Var.AC4 * ((unsigned long)(Var.X3 + 32768)) / (2 << 15);
	Var.B7 = ((unsigned long)Var.UP - Var.B3) * (50000 >> Var.OSS);

	if (Var.B7 < 0x80000000u)
	{
		Var.P = (Var.B7 * 2) / Var.B4;
	}
	else
	{
		Var.P = (Var.B7 / Var.B4) * 2;
	}

	Var.X1 = (Var.P / (2 << 8)) * (Var.P / (2 << 8));
	Var.X1 = (Var.X1 * 3038) / (2 << 16);
	Var.X2 = (-7357 * Var.P) / (2 << 16);

	Var.P = Var.P + (Var.X1 + Var.X2 + 3791) / (2 << 4);
	StatusTemp = APT_DataStatus_t::Available;
	StatusPress = APT_DataStatus_t::Available;

	Status = APT_Status_t::NextTask;
}

int APT_AirPressureTemp::Update()
{
	switch (State)
	{
	case APT_State_t::Initialize:
		Initialize();
		State = APT_State_t::Request_UncompTemp;
		Status = APT_Status_t::Idle;
		break;
	case APT_State_t::Request_UncompTemp:
		Request_UncompTemp();
		if (Status == APT_Status_t::NextTask)
		{
			State = APT_State_t::Request_UncompPress;
			Status = APT_Status_t::Idle;
		}
		break;
	case APT_State_t::Request_UncompPress:
		Request_UncompPress();
		if (Status == APT_Status_t::NextTask)
		{
			State = APT_State_t::ProcessResults;
			Status = APT_Status_t::Idle;
		}
		break;
	case APT_State_t::ProcessResults:
		ProcessResults();
		if (Status == APT_Status_t::NextTask)
		{
			State = APT_State_t::Request_UncompTemp;
			Status = APT_Status_t::Idle;
		}
		break;
	default:
		break;
	}
	return 0;
}

long APT_AirPressureTemp::PrintTemperature()
{
	switch (StatusTemp)
	{
	case APT_DataStatus_t::Unavailable:
		return -1;
		break;		
	case APT_DataStatus_t::Available:
		StatusTemp = APT_DataStatus_t::Unavailable;
		return Var.T;
		break;
	default:
		return -1;
		break;
	}
	return -1;	
}

long APT_AirPressureTemp::PrintAirPressure()
{
	switch (StatusPress)
	{
	case APT_DataStatus_t::Unavailable:
		return -1;
		break;
	case APT_DataStatus_t::Available:
		StatusPress = APT_DataStatus_t::Unavailable;
		return Var.P;
		break;
	default:
		return -1;
		break;
	}
	return -1;
}
