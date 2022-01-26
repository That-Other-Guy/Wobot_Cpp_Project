#pragma once
#ifndef BMP085_H_
#define BMP085_H_

#include <cstdint>

enum class BMP085_Bit_t : uint8_t
{
	Unset = 0x0,
	Set = 0x1,
};

enum class BMP085_DevAddr_t : uint8_t
{
	Default = 0x77u,
};

enum class BMP085_OSS_t
{
	UltraLowPower = 0,
	Standard = 1,
	HighResolution = 2,
	UltraHighResolution = 3,
};

enum class BMP085_RegAddr_t : uint8_t
{
	AC1_MSB = 0xAAu,
	AC1_LSB = 0xABu,
				
	AC2_MSB = 0xACu,
	AC2_LSB = 0xADu,
				
	AC3_MSB = 0xAEu,
	AC3_LSB = 0xAFu,
				
	AC4_MSB = 0xB0u,
	AC4_LSB = 0xB1u,
				
	AC5_MSB = 0xB2u,
	AC5_LSB = 0xB3u,
				
	AC6_MSB = 0xB4u,
	AC6_LSB = 0xB5u,

	B1_MSB = 0xB6u,
	B1_LSB = 0xB7u,

	B2_MSB = 0xB8u,
	B2_LSB = 0xB9u,

	MB_MSB = 0xBAu,
	MB_LSB = 0xBBu,

	MC_MSB = 0xBCu,
	MC_LSB = 0xBDu,

	MD_MSB = 0xBEu,
	MD_LSB = 0xBFu,

	COMMAND = 0xF4u,

	DATA_MSB = 0xF6u,
	DATA_LSB = 0xF7u,
	DATA_XLSB = 0xF8u,
};

enum class BMP085_Command_t
{
	RequestTemp = 0x2Eu,
	RequestPress_ULP = 0x34u,
	RequestPress_STD = 0x74u,
	RequestPress_HR = 0xB4u,
	RequestPress_UHR = 0xF4u,
};

struct BMP085_Data_s
{
	uint8_t BMP085_AC1_MSB;
	uint8_t BMP085_AC1_LSB;
	
	uint8_t BMP085_AC2_MSB;
	uint8_t BMP085_AC2_LSB;
	
	uint8_t BMP085_AC3_MSB;
	uint8_t BMP085_AC3_LSB;
	
	uint8_t BMP085_AC4_MSB;
	uint8_t BMP085_AC4_LSB;
	
	uint8_t BMP085_AC5_MSB;
	uint8_t BMP085_AC5_LSB;
	
	uint8_t BMP085_AC6_MSB;
	uint8_t BMP085_AC6_LSB;
	
	uint8_t BMP085_B1_MSB;
	uint8_t BMP085_B1_LSB;
	
	uint8_t BMP085_B2_MSB;
	uint8_t BMP085_B2_LSB;
	
	uint8_t BMP085_MB_MSB;
	uint8_t BMP085_MB_LSB;
	
	uint8_t BMP085_MC_MSB;
	uint8_t BMP085_MC_LSB;
	
	uint8_t BMP085_MD_MSB;
	uint8_t BMP085_MD_LSB;
	
	uint8_t BMP085_DATA_MSB;
	uint8_t BMP085_DATA_LSB;
	uint8_t BMP085_DATA_XLSB;
};
#endif /* BMP085_H_ */