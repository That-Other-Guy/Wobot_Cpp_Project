#include "gyroscope.h"
#include "wiringPiI2C.h"

Gyroscope::Gyroscope()
{
	Sm = Gyro_Sm::Initialize;
	Data = { 0 };
	fscaling_factor_x = ((2000.0f * 2.0f * 3.1415f) / 360.0f) / (float)(32767.0f);
	fscaling_factor_y = ((2000.0f * 2.0f * 3.1415f) / 360.0f) / (float)(32767.0f);
	fscaling_factor_z = ((2000.0f * 2.0f * 3.1415f) / 360.0f) / (float)(32767.0f);
}

Gyroscope::~Gyroscope()
{
}

void Gyroscope::Initialize()
{
	i2c_device_handle = wiringPiI2CSetup((int)ITG3200_DeviceAddress::addADXL345);
	uint8_t cfg = ((uint8_t)ITG3200_RW_CF_SEL::_2000_deg_sec | (uint8_t)ITG3200CF_DLPF_CFG::_020Hz);
	wiringPiI2CWriteReg8(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_DLPF_FS, (int)cfg);
	return;
}

void Gyroscope::ProcessResults()
{
	uint8_t raw_t_h = (uint8_t)wiringPiI2CReadReg16(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_TEMP_OUT_H);
	uint8_t raw_t_l = (uint8_t)wiringPiI2CReadReg16(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_TEMP_OUT_L);
						   
	uint8_t raw_x_h = (uint8_t)wiringPiI2CReadReg16(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_GYRO_XOUT_H);
	uint8_t raw_x_l = (uint8_t)wiringPiI2CReadReg16(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_GYRO_XOUT_L);
						   
	uint8_t raw_y_h = (uint8_t)wiringPiI2CReadReg16(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_GYRO_YOUT_H);
	uint8_t raw_y_l = (uint8_t)wiringPiI2CReadReg16(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_GYRO_YOUT_L);
						   
	uint8_t raw_z_h = (uint8_t)wiringPiI2CReadReg16(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_GYRO_ZOUT_H);
	uint8_t raw_z_l = (uint8_t)wiringPiI2CReadReg16(i2c_device_handle, (int)ITG3200_RegisterAddress::regITG3200_GYRO_ZOUT_L);

	Data.DATA_TEMP = (int16_t)((raw_t_l & 0x00FF) | ((raw_t_h & 0xFF00) >> 8));
	Data.DATA_X = (int16_t)((raw_x_l & 0x00FF) | ((raw_x_h & 0xFF00) >> 8));
	Data.DATA_Y = (int16_t)((raw_y_l & 0x00FF) | ((raw_y_h & 0xFF00) >> 8));
	Data.DATA_Z = (int16_t)((raw_z_l & 0x00FF) | ((raw_z_h & 0xFF00) >> 8));

	G_Data.X = (float)Data.DATA_X * fscaling_factor_x;
	G_Data.Y = (float)Data.DATA_Y * fscaling_factor_y;
	G_Data.Z = (float)Data.DATA_Z * fscaling_factor_z;
}

GyroData Gyroscope::Update()
{
	switch (Sm)
	{
	case Gyro_Sm::Initialize:
		Initialize();
		Sm = Gyro_Sm::ProcessResults;
		Status = Gyro_Status::Idle;
		break;
	case Gyro_Sm::ProcessResults:
		ProcessResults();
		break;
	default:
		break;
	}
	return G_Data;
}
