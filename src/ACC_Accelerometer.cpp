#include "ACC_Accelerometer.h"
#include "wiringPi.h"
#include "wiringPiI2C.h"

#include <cstring>

ACC_Accelerometer::ACC_Accelerometer(ADXL345_Rate_t _Rate, ADXL345_Gain_t _Gain)
{	
	State = ACC_State_t::Initialize;
	memset(&AccelData, 0, sizeof(ACC_Data_s));

	switch (_Rate)
	{
	case ADXL345_Rate_t::_3200Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_3200;
		break;
	case ADXL345_Rate_t::_1600Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_1600;
		break;
	case ADXL345_Rate_t::_800Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_800;
		break;
	case ADXL345_Rate_t::_400Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_400;
		break;
	case ADXL345_Rate_t::_200Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_200;
		break;
	case ADXL345_Rate_t::_100Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_100;
		break;
	case ADXL345_Rate_t::_50Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_50;
		break;
	case ADXL345_Rate_t::_25Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_25;
		break;
	case ADXL345_Rate_t::_12_5Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_12_5;
		break;
	case ADXL345_Rate_t::_6_25Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_6_25;
		break;
	case ADXL345_Rate_t::_3_13Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_3_13;
		break;
	case ADXL345_Rate_t::_1_56Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_1_56;
		break;
	case ADXL345_Rate_t::_0_78Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_0_78;
		break;
	case ADXL345_Rate_t::_0_39Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_0_39;
		break;
	case ADXL345_Rate_t::_0_20Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_0_20;
		break;
	case ADXL345_Rate_t::_0_10Hz:
		Data.BW_RATE = ADXL345_CF_BW_RATE_t::Rate_0_10;
		break;
	default:
		break;
	}
	
	switch (_Gain)
	{
	case ADXL345_Gain_t::_2g:
		fscaling_factor = 9.8f / 256;
		break;
	case ADXL345_Gain_t::_4g:
		fscaling_factor = 9.8f / 128;
		break;
	case ADXL345_Gain_t::_8g:
		fscaling_factor = 9.8f / 64;
		break;
	case ADXL345_Gain_t::_16g:
		fscaling_factor = 9.8f / 32;
		break;
	case ADXL345_Gain_t::_FullRes:
		fscaling_factor = 9.8f / 256;
		break;
	default:
		break;
	}

}

ACC_Accelerometer::~ACC_Accelerometer()
{
	return;
}

void ACC_Accelerometer::Initialize()
{
	fd_i2c_device = wiringPiI2CSetup((int)ADXL345_DevAddr_t::Default);

	Data.POWER_CTL = ADXL345_CF_POWER_CTL_t::Measure;
	uint16_t cfg = (uint16_t)(((uint8_t)(Data.POWER_CTL) << 8) | (uint8_t)Data.BW_RATE);
	wiringPiI2CWriteReg16(fd_i2c_device, (int)ADXL345_RegAddr_t::BW_RATE, (int)cfg);	
	return;
}

void ACC_Accelerometer::ProcessResults()
{
	Data.DATA_X = (int16_t)wiringPiI2CReadReg16(fd_i2c_device, (int)ADXL345_RegAddr_t::DATAX0);
	Data.DATA_Y = (int16_t)wiringPiI2CReadReg16(fd_i2c_device, (int)ADXL345_RegAddr_t::DATAY0);
	Data.DATA_Z = (int16_t)wiringPiI2CReadReg16(fd_i2c_device, (int)ADXL345_RegAddr_t::DATAZ0);

	AccelData.fX = (float)Data.DATA_X * fscaling_factor;
	AccelData.fY = (float)Data.DATA_Y * fscaling_factor;
	AccelData.fZ = (float)Data.DATA_Z * fscaling_factor;

	return;
}

ACC_Data_s ACC_Accelerometer::Update()
{
	switch (State)
	{
	case ACC_State_t::Initialize:
		Initialize();
		State = ACC_State_t::ProcessResults;
		Status = ACC_Status_t::Idle;
		break;
	case ACC_State_t::ProcessResults:
		ProcessResults();
		break;
	default:
		break;
	}
	return AccelData;
}
