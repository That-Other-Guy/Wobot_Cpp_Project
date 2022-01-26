#pragma once

#ifndef ITG3200_H_
#define ITG3200_H_

#include <cstdint>

enum class ITG3200_Bool : uint8_t
{
	bitITG3200_Unset = 0x0u,
	bitITG3200_Set = 0x1u,
};

enum class ITG3200_DeviceAddress : uint8_t
{
	addADXL345 = 0x68u,
};

enum class ITG3200_RegisterAddress : uint8_t
{
	regITG3200_WHO_AM_I = 0x00u,

	regITG3200_SMPLRT_DIV = 0x15u,
	regITG3200_DLPF_FS = 0x16,
	regITG3200_INT_CFG = 0x17,

	regITG3200_INT_STATUS = 0x1A,
	regITG3200_TEMP_OUT_H = 0x1B,
	regITG3200_TEMP_OUT_L = 0x1C,
	regITG3200_GYRO_XOUT_H = 0x1D,
	regITG3200_GYRO_XOUT_L = 0x1E,
	regITG3200_GYRO_YOUT_H = 0x1F,
	regITG3200_GYRO_YOUT_L = 0x20,
	regITG3200_GYRO_ZOUT_H = 0x21,
	regITG3200_GYRO_ZOUT_L = 0x22,
	regITG3200_PWR_MGM = 0x3E,
};

enum class ITG3200_RW_CF_SEL : uint8_t
{
	Not_Used_0 = 0x00u,
	Not_Used_1 = 0x08u,
	Not_Used_2 = 0x10u,

	_2000_deg_sec = 0x18u,
};

enum class ITG3200CF_DLPF_CFG : uint8_t
{
	_256Hz = 0x00,
	_188Hz = 0x01,
	_098Hz = 0x02,
	_042Hz = 0x03,

	_020Hz = 0x04,
	_010Hz = 0x05,
	_005Hz = 0x06,
	Reserved_7 = 0x07,
};

enum class ITG3200_CF_INT_CFG : uint8_t
{
	RAW_RDY_EN = 0x01,
	NotUsed_D1 = 0x02u,
	ITG_RDY_EN = 0x04u,
	NotUsed_D3 = 0x08u,

	INT_ANYRD_2CLEAR = 0x10u,
	LATCH_INT_EN = 0x20u,
	OPEN = 0x40u,
	ACTL = 0x80u,
};

enum class ITG3200_R_INT_CFG : uint8_t
{
	RAW_DATA_RDY = 0x01u,
	ITG_RDY = 0x04u,
};

enum class ITG3200_CF_PWR_MGM : uint8_t
{
	Internal_Oscillator = 0x00u,
	PLL_with_X_gyro = 0x01u,
	PLL_with_Y_gyro = 0x02u,
	PLL_with_Z_gyro = 0x03u,

	PLL_with_EXT_32_768kHz = 0x04u,
	PLL_with_EXT_19_2Mhz = 0x05u,

	STBY_ZG = 0x08u,
	STBY_YG = 0x10u,
	STBY_XG = 0x20u,

	SLEEP = 0x40u,
	H_RESET = 0x80u,
};

struct ITG3200_Data
{
	uint8_t WHO_AM_I;	
	uint8_t SMPLRT_DIV;
	uint8_t DLPF_FS;
	uint8_t INT_CFG;	
	uint8_t INT_STATUS;

	int16_t DATA_TEMP;
	int16_t DATA_X;
	int16_t DATA_Y;
	int16_t DATA_Z;

	uint8_t PWR_MGM;
};
#endif /* ITG3200_H_ */