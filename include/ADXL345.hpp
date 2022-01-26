#pragma once

#ifndef ADXL345_H_
#define ADXL345_H_

#include <cstdint>

enum class ADXL345_Bit_t : uint8_t
{
	Unset = 0x0u,
	Set = 0x1u,
};

enum class ADXL345_DevAddr_t : uint8_t
{
	Default = 0x53u,
};

enum class ADXL345_Gain_t
{
	_2g,
	_4g,	
	_8g,
	_16g,
	_FullRes
};

enum class ADXL345_Rate_t
{
	_3200Hz,
	_1600Hz,
	_800Hz,
	_400Hz,
	  
	_200Hz,
	_100Hz,
	_50Hz,
	_25Hz,
	   
	_12_5Hz,
	_6_25Hz,
	_3_13Hz,
	_1_56Hz,
	 
	_0_78Hz,
	_0_39Hz,
	_0_20Hz,
	_0_10Hz,
};

enum class ADXL345_RegAddr_t : uint8_t
{
	DEVID = 0x00u,
	THRESH_TAP = 0x1Du,
	OFSX = 0x1Eu,
	OFSY = 0x1Fu,
	OFSZ = 0x20u,
	DUR = 0x21u,
	LATENT = 0x22u,
	WINDOW = 0x23u,
	THRESH_ACT = 0x24u,
	THRESH_INACT = 0x25u,
	TIME_INACT = 0x26u,
	ACT_INACT_CTL = 0x27u,
	THRESH_FF = 0x28u,
	TIME_FF = 0x29u,
	TAP_AXES = 0x2Au,
	TACT_TAP_STATUS = 0x2Bu,
	BW_RATE = 0x2Cu,
	POWER_CTL = 0x2Du,
	INT_ENABLE = 0x2Eu,
	INT_MAP = 0x2Fu,
	INT_SOURCE = 0x30u,
	DATA_FORMAT = 0x31u,
	DATAX0 = 0x32u,
	DATAX1 = 0x33u,
	DATAY0 = 0x34u,
	DATAY1 = 0x35u,
	DATAZ0 = 0x36u,
	DATAZ1 = 0x37u,
	FIFO_CTL = 0x38u,
	FIFO_STATUS = 0x39u,
};

enum class ADXL345_CF_ACT_INACT_CTL_t : uint8_t
{
	ACT_acdc = 0x80u,
	ACT_X_enable = 0x40u,
	ACT_Y_enable = 0x20u,
	ACT_Z_enable = 0x10u,
	INACT_acdc = 0x08u,
	INACT_X_enable = 0x04u,
	INACT_Y_enable = 0x02u,
	INACT_Z_enable = 0x01u,	
};

enum class ADXL345_CF_TAP_AXES_t : uint8_t
{
	Not_Used_D7 = 0x80u,
	Not_Used_D6 = 0x40u,
	Not_Used_D5 = 0x20u,
	Not_Used_D4 = 0x10u,
	Suppress = 0x08u,
	TAP_X_enable = 0x04u,
	TAP_Y_enable = 0x02u,
	TAP_Z_enable = 0x01u,
};

enum class ADXL345_CF_ACT_TAP_STATUS_t : uint8_t
{
	Not_Used_D7 = 0x80u,
	ACT_X = 0x40u,
	ACT_Y = 0x20u,
	ACT_Z = 0x10u,
	Asleep = 0x08u,
	TAP_X = 0x04u,
	TAP_Y = 0x02u,
	TAP_Z = 0x01u,
};

enum class ADXL345_CF_BW_RATE_t : uint8_t
{
	Not_Used_D7 = 0x80u,
	Not_Used_D6 = 0x40u,
	Not_Used_D5 = 0x20u,
	LOW_POWER = 0x10u,

	Rate_3200 = 0x0Fu,
	Rate_1600 = 0x0Eu,
	Rate_800 = 0x0Du,
	Rate_400 = 0x0Cu,

	Rate_200 = 0x0Bu,
	Rate_100 = 0x0Au,
	Rate_50 = 0x09u,
	Rate_25 = 0x08u,

	Rate_12_5 = 0x07u,
	Rate_6_25 = 0x06u,
	Rate_3_13 = 0x05u,
	Rate_1_56 = 0x04u,

	Rate_0_78 = 0x03u,
	Rate_0_39 = 0x02u,
	Rate_0_20 = 0x01u,
	Rate_0_10 = 0x00u,	
};

enum class ADXL345_CF_POWER_CTL_t : uint8_t
{
	Not_Used_D7 = 0x80u,
	Not_Used_D6 = 0x40u,
	Link = 0x20u,
	AUTO_SLEEP = 0x10u,
	Measure = 0x08u,
	Sleep = 0x04u,

	Wakeup_1Hz = 0x03u,
	Wakeup_2Hz = 0x02u,
	Wakeup_4Hz = 0x01u,
	Wakeup_8Hz = 0x00u,	
};

enum class ADXL345_CF_INT_ENABLE_t : uint8_t
{
	DATA_READY = 0x80u,
	SINGLE_TAP = 0x40u,
	DOUBLE_TAP = 0x20u,
	Activity = 0x10u,
	Inactivity = 0x08u,
	FREE_FALL = 0x04u,
	Watermark = 0x02u,
	Overrun = 0x01u,
};

enum class ADXL345_CF_INT_MAP_t : uint8_t
{
	DATA_READY = 0x80u,
	SINGLE_TAP = 0x40u,
	DOUBLE_TAP = 0x20u,
	Activity = 0x10u,
	Inactivity = 0x08u,
	FREE_FALL = 0x04u,
	Watermark = 0x02u,
	Overrun = 0x01u,
};

enum class ADXL345_CF_INT_SOURCE_t : uint8_t
{
	DATA_READY = 0x80u,
	SINGLE_TAP = 0x40u,
	DOUBLE_TAP = 0x20u,
	Activity = 0x10u,
	Inactivity = 0x08u,
	FREE_FALL = 0x04u,
	Watermark = 0x02u,
	Overrun = 0x01u,
};

enum class ADXL345_CF_DATA_FORMAT_t : uint8_t
{
	SELF_TEST = 0x80u,
	SPI = 0x40u,
	INT_INVERT = 0x20u,
	Not_Used_D4 = 0x10u,
	FULL_RES = 0x08u,
	Justify = 0x04u,

	Range_16g = 0x03u,
	Range_8g = 0x02u,
	Range_4g = 0x01u,
	Range_2g = 0x00u,
};

enum class ADXL345_CF_FIFO_CTL_t : uint8_t
{
	FIFO_MODE_Bypass = 0x00u,
	FIFO_MODE_FIFO = 0x40u,
	FIFO_MODE_Stream = 0x80u,
	FIFO_MODE_Trigger = 0xC0u,

	Trigger = 0x20u,

	Samples_D4 = 0x10,
	Samples_D3 = 0x08,
	Samples_D2 = 0x04,
	Samples_D1 = 0x02,
	Samples_D0 = 0x01,
};

enum class ADXL345_CF_FIFO_STATUS_t : uint8_t
{
	FIFO_TRIG = 0x80u,
	Not_Used_D6 = 0x40u,

	Entries_D5 = 0x20,
	Entries_D4 = 0x10,
	Entries_D3 = 0x08,
	Entries_D2 = 0x04,
	Entries_D1 = 0x02,
	Entries_D0 = 0x01,
};

struct ADXL345_Data_s
{
	uint8_t THRESH_TAP;
	uint8_t OFS_X;
	uint8_t OFS_Y;
	uint8_t OFS_Z;
	uint8_t DUR;
	uint8_t Latent;
	uint8_t Window;
	uint8_t THRESH_ACT;
	uint8_t THRESH_INACT;
	uint8_t TIME_INACT;
	
	ADXL345_CF_ACT_INACT_CTL_t ACT_INACT_CTL;
	
	uint8_t THRESH_FF;
	uint8_t TIME_FF;
	uint8_t TAP_AXES;
	uint8_t ACT_TAP_STATUS;
	
	ADXL345_CF_BW_RATE_t BW_RATE;
	ADXL345_CF_POWER_CTL_t POWER_CTL;
	ADXL345_CF_INT_ENABLE_t INT_ENABLE;
	ADXL345_CF_INT_MAP_t INT_MAP;
	ADXL345_CF_INT_SOURCE_t INT_SOURCE;
	ADXL345_CF_DATA_FORMAT_t DATA_FORMAT;

	int16_t DATA_X;
	int16_t DATA_Y;
	int16_t DATA_Z;

	ADXL345_CF_FIFO_CTL_t FIFO_CTL;
	ADXL345_CF_FIFO_STATUS_t FIFO_STATUS;
};
#endif /* ADXL345_H_ */