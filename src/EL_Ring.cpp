#include "EL_Ring.hpp"
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include <cerrno>
#include <cstdio>

EL_Ring::EL_Ring(EL_Position_t _Pos, W_COut_t _COut)
{
	Status = EL_Status_t::Init;
	Position = _Pos;
	COut = _COut;

	/*
	* Load pixel register values
	* LED_WRL_POSITION_A = 0x16u
	* LED_WRL_POSITION_B = 0x1Eu
	* LED_WRL_POSITION_C = 0x22u
	* LED_WRL_POSITION_D = 0x3Au
	* LED_WRL_POSITION_E = 0x42u
	* LED_WRL_POSITION_F = 0x3Eu
	* LED_WRL_POSITION_G = 0x36u
	* LED_WRL_POSITION_H = 0x1Au
	* 
	* LED_RRL_POSITION_A = 0x0Au
	* LED_RRL_POSITION_B = 0x08u
	* LED_RRL_POSITION_C = 0x0Eu
	* LED_RRL_POSITION_D = 0x26u
	* LED_RRL_POSITION_E = 0x2Au
	* LED_RRL_POSITION_F = 0x2Eu
	* LED_RRL_POSITION_G = 0x32u
	* LED_RRL_POSITION_H = 0x12u
	*/

	switch (_Pos)
	{	
	case EL_Position_t::Left:
		Pixel_A.Red_Register = PCA9685_RegAddr_t::LED1_OFF_L;
		Pixel_B.Red_Register = PCA9685_RegAddr_t::LED0_OFF_L;
		Pixel_C.Red_Register = PCA9685_RegAddr_t::LED2_OFF_L;
		Pixel_D.Red_Register = PCA9685_RegAddr_t::LED8_OFF_L;
		Pixel_E.Red_Register = PCA9685_RegAddr_t::LED9_OFF_L;
		Pixel_F.Red_Register = PCA9685_RegAddr_t::LEDA_OFF_L;
		Pixel_G.Red_Register = PCA9685_RegAddr_t::LEDB_OFF_L;
		Pixel_H.Red_Register = PCA9685_RegAddr_t::LED3_OFF_L;

		Pixel_A.White_Register = PCA9685_RegAddr_t::LED4_OFF_L;
		Pixel_B.White_Register = PCA9685_RegAddr_t::LED6_OFF_L;
		Pixel_C.White_Register = PCA9685_RegAddr_t::LED7_OFF_L;
		Pixel_D.White_Register = PCA9685_RegAddr_t::LEDD_OFF_L;
		Pixel_E.White_Register = PCA9685_RegAddr_t::LEDF_OFF_L;
		Pixel_F.White_Register = PCA9685_RegAddr_t::LEDE_OFF_L;
		Pixel_G.White_Register = PCA9685_RegAddr_t::LEDC_OFF_L;
		Pixel_H.White_Register = PCA9685_RegAddr_t::LED5_OFF_L;
		break;
	case EL_Position_t::Right:
		Pixel_A.Red_Register = PCA9685_RegAddr_t::LED9_OFF_L;
		Pixel_B.Red_Register = PCA9685_RegAddr_t::LEDA_OFF_L;
		Pixel_C.Red_Register = PCA9685_RegAddr_t::LEDB_OFF_L;
		Pixel_D.Red_Register = PCA9685_RegAddr_t::LED3_OFF_L;
		Pixel_E.Red_Register = PCA9685_RegAddr_t::LED1_OFF_L;
		Pixel_F.Red_Register = PCA9685_RegAddr_t::LED0_OFF_L;
		Pixel_G.Red_Register = PCA9685_RegAddr_t::LED2_OFF_L;
		Pixel_H.Red_Register = PCA9685_RegAddr_t::LED8_OFF_L;

		Pixel_A.White_Register = PCA9685_RegAddr_t::LEDF_OFF_L;
		Pixel_B.White_Register = PCA9685_RegAddr_t::LEDE_OFF_L;
		Pixel_C.White_Register = PCA9685_RegAddr_t::LEDC_OFF_L;
		Pixel_D.White_Register = PCA9685_RegAddr_t::LED5_OFF_L;
		Pixel_E.White_Register = PCA9685_RegAddr_t::LED4_OFF_L;
		Pixel_F.White_Register = PCA9685_RegAddr_t::LED6_OFF_L;
		Pixel_G.White_Register = PCA9685_RegAddr_t::LED7_OFF_L;
		Pixel_H.White_Register = PCA9685_RegAddr_t::LEDD_OFF_L;		
		break;
	}

	Pixel_A.H = EL_RingPosH_t::A;
	Pixel_A.V = EL_RingPosV_t::A;
	Pixel_B.H = EL_RingPosH_t::B;
	Pixel_B.V = EL_RingPosV_t::B;
	Pixel_C.H = EL_RingPosH_t::C;
	Pixel_C.V = EL_RingPosV_t::C;
	Pixel_D.H = EL_RingPosH_t::D;
	Pixel_D.V = EL_RingPosV_t::D;
	Pixel_E.H = EL_RingPosH_t::E;
	Pixel_E.V = EL_RingPosV_t::E;
	Pixel_F.H = EL_RingPosH_t::F;
	Pixel_F.V = EL_RingPosV_t::F;
	Pixel_G.H = EL_RingPosH_t::G;
	Pixel_G.V = EL_RingPosV_t::G;
	Pixel_H.H = EL_RingPosH_t::H;
	Pixel_H.V = EL_RingPosV_t::H;

	PixelArray[0] = &Pixel_A;
	PixelArray[1] = &Pixel_B;
	PixelArray[2] = &Pixel_C;
	PixelArray[3] = &Pixel_D;
	PixelArray[4] = &Pixel_E;
	PixelArray[5] = &Pixel_F;
	PixelArray[6] = &Pixel_G;
	PixelArray[7] = &Pixel_H;
	
	Task = EL_Action_t::None;
	State = EL_State_t::Closed;
	SetPoi.H = EL_PoiAngle_t::Centre;
	SetPoi.V = EL_PoiAngle_t::Centre;

	Poi.H = EL_PoiAngle_t::Centre;
	Poi.V = EL_PoiAngle_t::Centre;

	SetColour = EL_Colours_t::White;
	SetGain = EL_Gain_t::Full;

	memset(&DisplayData, (int)(EL_Gain_t::Off), sizeof(DisplayData));

	for (int pixel_index = 0; pixel_index < 8; pixel_index++)
	{
		PixelArray[pixel_index]->Status = EL_Status_t::Update;
	}
	return;
}

EL_Ring::~EL_Ring() {}

EL_Status_t EL_Ring::InitDriver()
{
	switch (Position)
	{
	case EL_Position_t::Left:
		fd_i2c_dev = wiringPiI2CSetup((int)PCA9685_DevAddr_t::LeftDevice);
		//printf("%d:\n%d:\n", fd_i2c_dev, errno);		
		if (fd_i2c_dev < 0)
		{
			if (COut != W_COut_t::W_Silent)
			{
				printf("Failed to Setup Leftt PCA9685 Device.\n %s\n", strerror(errno));
			}
			return EL_Status_t::Init;
		}
		break;
	case EL_Position_t::Right:
		fd_i2c_dev = wiringPiI2CSetup((int)PCA9685_DevAddr_t::RightDevice);
		//printf("%d:\n%d:\n", fd_i2c_dev, errno);		
		if (fd_i2c_dev < 0)
		{
			if (COut != W_COut_t::W_Silent)
			{
				printf("Failed to Setup Right PCA9685 Device.\n %s\n", strerror(errno));
			}
			return EL_Status_t::Init;
		}
		break;
	default:
		break;
	}
	// Setup Hardware
	PCA9685_Mode_s DevMode;

	DevMode.ALLCALL = PCA9685_Bit_t::Unset;
	DevMode.SUB3 = PCA9685_Bit_t::Unset;
	DevMode.SUB2 = PCA9685_Bit_t::Unset;
	DevMode.SUB1 = PCA9685_Bit_t::Unset;
	DevMode.SLEEP = PCA9685_Bit_t::Unset;
	DevMode.AI = PCA9685_Bit_t::Set;
	DevMode.EXTCLK = PCA9685_Bit_t::Unset;
	DevMode.RESTART = PCA9685_Bit_t::Set;

	DevMode.OUTNE0 = PCA9685_Bit_t::Unset;
	DevMode.OUTNE1 = PCA9685_Bit_t::Unset;
	DevMode.OUTDRV = PCA9685_Bit_t::Unset;
	DevMode.OCH = PCA9685_Bit_t::Unset;
	DevMode.INVRT = PCA9685_Bit_t::Set;
	DevMode.Reserved5 = PCA9685_Bit_t::Unset;
	DevMode.Reserved6 = PCA9685_Bit_t::Unset;
	DevMode.Reserved7 = PCA9685_Bit_t::Unset;

	int return_value = wiringPiI2CWriteReg16(fd_i2c_dev, (int)PCA9685_RegAddr_t::MODE1, (int)DevMode.raw);
	if (return_value < 0)
	{
		if (COut != W_COut_t::W_Silent)
		{
			if (Position == EL_Position_t::Right)
			{
				printf("Failed to Write Mode Settings to Right PCA9685 Device.\n %s\n", strerror(errno));
				return EL_Status_t::Init;
			}
			else
			{
				printf("Failed to Write Mode Settings to Left PCA9685 Device.\n %s\n", strerror(errno));
				return EL_Status_t::Init;
			}
		}
	}

	return_value = wiringPiI2CWriteReg16(fd_i2c_dev, (int)PCA9685_RegAddr_t::ALL_LED_ON_L, (int)0x0000u);
	if (return_value < 0)
	{
		if (COut != W_COut_t::W_Silent)
		{
			if (Position == EL_Position_t::Right)
			{
				printf("Failed to Write to Right PCA9685 Device.\n %s\n", strerror(errno));
				return EL_Status_t::Init;
			}
			else
			{
				printf("Failed to Write to Left PCA9685 Device.\n %s\n", strerror(errno));
				return EL_Status_t::Init;
			}
		}
	}
	return_value = wiringPiI2CWriteReg16(fd_i2c_dev, (int)PCA9685_RegAddr_t::ALL_LED_OFF_L, (int)0x0000u);
	if (return_value < 0)
	{
		if (COut != W_COut_t::W_Silent)
		{			
			if (Position == EL_Position_t::Right)
			{
				printf("Failed to Write to Right PCA9685 Device.\n %s\n", strerror(errno));
				return EL_Status_t::Init;
			}
			else
			{
				printf("Failed to Write to Left PCA9685 Device.\n %s\n", strerror(errno));
				return EL_Status_t::Init;
			}
			
		}
	}
	if (COut != W_COut_t::W_Silent)
	{
		if (Position == EL_Position_t::Right)
		{
			printf("Right PCA9685 Device OK. fd-> %d\n", fd_i2c_dev);
		}
		else
		{
			printf("Left PCA9685 Device OK. fd-> %d\n", fd_i2c_dev);
		}
	}
	refresh_timer_time = micros();
	return EL_Status_t::Good;
}

void EL_Ring::SetPixel(int _h, int _v, EL_Gain_t _r, EL_Gain_t _w)
{	
	if (_h < 0)
		return;
	if (_h > 8)
		return;
	if (_v < 0)
		return;
	if (_v > 8)
		return;

	DisplayData[_h][_v].RedGain = _r;
	DisplayData[_h][_v].WhiteGain = _w;	
	return;
}

void EL_Ring::SetPixel(int _h, int _v)
{
	if (_h < 0)
		return;
	if (_h > 8)
		return;
	if (_v < 0)
		return;
	if (_v > 8)
		return;

	switch (SetColour)
	{
	case EL_Colours_t::White:
		DisplayData[_h][_v].RedGain = EL_Gain_t::Off;
		DisplayData[_h][_v].WhiteGain = SetGain;
		return; break;		
	case EL_Colours_t::Red:
		DisplayData[_h][_v].RedGain = SetGain;
		DisplayData[_h][_v].WhiteGain = EL_Gain_t::Off;
		return; break;
	case EL_Colours_t::Pink:
		DisplayData[_h][_v].WhiteGain = SetGain;
		DisplayData[_h][_v].RedGain = SetGain;
		return; break;
	default:
		return; break;
	}	
	return;
}

void EL_Ring::ResetPixel(int _h, int _v)
{
	if (_h < 0)
		return;
	if (_h > 8)
		return;
	if (_v < 0)
		return;
	if (_v > 8)
		return;
	
	DisplayData[_h][_v].RedGain = EL_Gain_t::Off;
	DisplayData[_h][_v].WhiteGain = EL_Gain_t::Off;
	return;
}

void EL_Ring::ResetAllPixels()
{
	for (int hi = 0; hi < 9; hi++)
		for (int vi = 0; vi < 9; vi++)
		{
			ResetPixel(hi, vi);
		}
	return;
}

void EL_Ring::SetAllPixels()
{
	for (int hi = 0; hi < 9; hi++)
		for (int vi = 0; vi < 9; vi++)
		{
			SetPixel(hi, vi);
		}
	return;
}

void EL_Ring::SetAllPixels(EL_Gain_t _Gain)
{
	EL_Gain_t GainCopy = SetGain;
	SetGain = _Gain;
	for (int hi = 0; hi < 9; hi++)
		for (int vi = 0; vi < 9; vi++)
		{
			SetPixel(hi, vi);
		}
	SetGain = GainCopy;
	return;
}

void EL_Ring::SetAllPixels(EL_Colours_t _Colour)
{
	EL_Colours_t ColourCopy = SetColour;
	SetColour = _Colour;
	for (int hi = 0; hi < 9; hi++)
		for (int vi = 0; vi < 9; vi++)
		{
			SetPixel(hi, vi);
		}
	SetColour = ColourCopy;
	return;
}

void EL_Ring::SetAllPixels(EL_Colours_t _Colour, EL_Gain_t _Gain)
{
	EL_Colours_t ColourCopy = SetColour;
	SetColour = _Colour;
	EL_Gain_t GainCopy = SetGain;
	SetGain = _Gain;

	for (int hi = 0; hi < 9; hi++)
		for (int vi = 0; vi < 9; vi++)
		{
			SetPixel(hi, vi);
		}

	SetColour = ColourCopy;
	SetGain = GainCopy;
	return;
}

void EL_Ring::DimPixel(int _h, int _v)
{
	if (_h < 0)
		return;
	if (_h > 8)
		return;
	if (_v < 0)
		return;
	if (_v > 8)
		return;

	EL_PixColour_c PixCpy = DisplayData[_h][_v];

	switch (PixCpy.WhiteGain)
	{
	case EL_Gain_t::Full:
		PixCpy.WhiteGain = EL_Gain_t::High;
		break;
	case EL_Gain_t::High:
		PixCpy.WhiteGain = EL_Gain_t::Med;
		break;
	case EL_Gain_t::Med:
		PixCpy.WhiteGain = EL_Gain_t::Low;
		break;
	case EL_Gain_t::Low:
		PixCpy.WhiteGain = EL_Gain_t::Min;
		break;
	default:
		PixCpy.WhiteGain = EL_Gain_t::Off;
		break;
	}

	switch (PixCpy.RedGain)
	{
	case EL_Gain_t::Full:
		PixCpy.RedGain = EL_Gain_t::High;
		break;
	case EL_Gain_t::High:
		PixCpy.RedGain = EL_Gain_t::Med;
		break;
	case EL_Gain_t::Med:
		PixCpy.RedGain = EL_Gain_t::Low;
		break;
	case EL_Gain_t::Low:
		PixCpy.RedGain = EL_Gain_t::Min;
		break;
	default:
		PixCpy.RedGain = EL_Gain_t::Off;
		break;
	}
	DisplayData[_h][_v] = PixCpy;
	return;
}

void EL_Ring::DDimPixel(int _h, int _v)
{
	DimPixel(_h, _v);
	DimPixel(_h, _v);
	return;
}

void EL_Ring::LidPosition_Open()
{
	switch (DispStyle)
	{
	case EL_DisplayStyle_t::Narrow:
		SetPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		DDimPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		if (Position == EL_Position_t::Right)
		{
			SetPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
			DDimPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
			//PixelC
			//PixelD 
			//PixelE
			//PixelF
			//PixelG
			//PixelH
		}
		else
		{
			//PixelB
			//PixelC
			//PixelD
			//PixelE			
			//PixelF
			//PixelG
			SetPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
			DDimPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		}
		break;
	default:
		break;
	}
	return;
}

void EL_Ring::LidPosition_A()
{
	switch (DispStyle)
	{
	case EL_DisplayStyle_t::Norm:
		ResetPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		ResetPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
		ResetPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
		ResetPixel((int)EL_RingPosH_t::D, (int)EL_RingPosV_t::D);
		DDimPixel((int)EL_RingPosH_t::E, (int)EL_RingPosV_t::E);
		ResetPixel((int)EL_RingPosH_t::F, (int)EL_RingPosV_t::F);
		ResetPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
		ResetPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		break;
	case EL_DisplayStyle_t::Narrow:
		ResetPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		ResetPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
		ResetPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);		
		if (Position == EL_Position_t::Right)
		{
			ResetPixel((int)EL_RingPosH_t::D, (int)EL_RingPosV_t::D);
			DDimPixel((int)EL_RingPosH_t::E, (int)EL_RingPosV_t::E);
			DDimPixel((int)EL_RingPosH_t::F, (int)EL_RingPosV_t::F);
			
		}
		else
		{
			DDimPixel((int)EL_RingPosH_t::D, (int)EL_RingPosV_t::D);
			DDimPixel((int)EL_RingPosH_t::E, (int)EL_RingPosV_t::E);			
			ResetPixel((int)EL_RingPosH_t::F, (int)EL_RingPosV_t::F);
		}
		ResetPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
		ResetPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);		
		break;
	default:
		break;
	}
	return;
}

void EL_Ring::LidPosition_B()
{
	switch (DispStyle)
	{
	case EL_DisplayStyle_t::Norm:
		ResetPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		ResetPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
		ResetPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
		DDimPixel((int)EL_RingPosH_t::D, (int)EL_RingPosV_t::D);
		DimPixel((int)EL_RingPosH_t::E, (int)EL_RingPosV_t::E);
		DDimPixel((int)EL_RingPosH_t::F, (int)EL_RingPosV_t::F);
		ResetPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
		ResetPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		break;
	case EL_DisplayStyle_t::Narrow:
		ResetPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		ResetPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
		
		if (Position == EL_Position_t::Right)
		{
			ResetPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
			DDimPixel((int)EL_RingPosH_t::D, (int)EL_RingPosV_t::D);
			DimPixel((int)EL_RingPosH_t::E, (int)EL_RingPosV_t::E);
			DimPixel((int)EL_RingPosH_t::F, (int)EL_RingPosV_t::F);
			DDimPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);			
		}
		else
		{
			DDimPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
			DimPixel((int)EL_RingPosH_t::D, (int)EL_RingPosV_t::D);
			DimPixel((int)EL_RingPosH_t::E, (int)EL_RingPosV_t::E);			
			DDimPixel((int)EL_RingPosH_t::F, (int)EL_RingPosV_t::F);
			ResetPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
		}
		ResetPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		break;
	default:
		break;
	}
}

void EL_Ring::LidPosition_C()
{
	switch (DispStyle)
	{
	case EL_DisplayStyle_t::Norm:
		ResetPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		ResetPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
		DDimPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
		DimPixel((int)EL_RingPosH_t::D, (int)EL_RingPosV_t::D);
		//Pixel_E
		DimPixel((int)EL_RingPosH_t::F, (int)EL_RingPosV_t::F);
		DDimPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
		ResetPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		break;
	case EL_DisplayStyle_t::Narrow:
		ResetPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);		
		if (Position == EL_Position_t::Right)
		{
			ResetPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
			DDimPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
			DimPixel((int)EL_RingPosH_t::D, (int)EL_RingPosV_t::D);
			//PixelE
			//PixelF
			DimPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
			DDimPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);			
		}
		else
		{
			DDimPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
			DimPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
			//PixelD
			//PixelE			
			DimPixel((int)EL_RingPosH_t::F, (int)EL_RingPosV_t::F);
			DDimPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
			ResetPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		}
		break;
	default:
		break;
	}
}

void EL_Ring::LidPosition_D()
{
	switch (DispStyle)
	{
	case EL_DisplayStyle_t::Norm:
		ResetPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		DDimPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
		DimPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
		//PixelD
		//PixelE
		//PixelF
		DimPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
		DDimPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		break;
	case EL_DisplayStyle_t::Narrow:
		DDimPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		if (Position == EL_Position_t::Right)
		{
			DDimPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
			DimPixel((int)EL_RingPosH_t::C, (int)EL_RingPosV_t::C);
			//PixelD 
			//PixelE
			//PixelF
			//PixelG
			DimPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		}
		else
		{
			DimPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
			//PixelC
			//PixelD
			//PixelE			
			//PixelF
			DimPixel((int)EL_RingPosH_t::G, (int)EL_RingPosV_t::G);
			DDimPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		}
		break;
	default:
		break;
	}
}

void EL_Ring::LidPosition_E()
{
	switch (DispStyle)
	{
	case EL_DisplayStyle_t::Norm:
		DDimPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		DimPixel((int)EL_RingPosH_t::B, (int)EL_RingPosV_t::B);
		//PixelC
		//PixelD
		//PixelE
		//PixelF
		//PixelG
		DimPixel((int)EL_RingPosH_t::H, (int)EL_RingPosV_t::H);
		break;
	default:
		break;
	}
}

void EL_Ring::LidPosition_F()
{
	switch (DispStyle)
	{
	case EL_DisplayStyle_t::Norm:
		DimPixel((int)EL_RingPosH_t::A, (int)EL_RingPosV_t::A);
		//PixelB
		//PixelC
		//PixelD
		//PixelE
		//PixelF
		//PixelG
		//PixelH
		break;
	default:
		break;
	}
}

void EL_Ring::ClampPoi(EL_PoiAngle_t& _H, EL_PoiAngle_t& _V)
{
	int _h = (int)_H;
	int _v = (int)_V;

	if (_v == 0)
	{
		if ((_h == 3) || (_h == 5))
		{
			_h = 4;
		}
		else if (_h == 2)
		{
			_h = 3; _v = 1;
		}
		else if (_h == 6)
		{
			_h = 5; _v = 1;
		}
		else if (_h < 2)
		{
			_h = 2; _v = 2;
		}
		else if (_h > 6)
		{
			_h = 6; _v = 2;
		}
	}
	else if (_v == 1)
	{
		if (_h <= 2)
		{
			_h = 2; _v = 2;
		}
		else if (_h >= 6)
		{
			_h = 6; _v = 2;
		}
	}
	else if (_v == 2)
	{
		if (_h == 1)
		{
			_h = 2;
		}
		else if (_h == 7)
		{
			_h = 6;
		}
		else if (_h == 0)
		{
			_h = 1; _v = 3;
		}
		else if (_h == 8)
		{
			_h = 7; _v = 3;
		}
	}
	else if ((_v == 3) || (_v == 5))
	{
		if ((_h == 0) || (_h == 8))
		{
			_v = 4;
		}
	}
	else if (_v == 6)
	{
		if (_h == 1)
		{
			_h = 2;
		}
		else if (_h == 7)
		{
			_h = 6;
		}
		else if (_h == 0)
		{
			_h = 1; _v = 5;
		}
		else if (_h == 8)
		{
			_h = 7; _v = 5;
		}
	}
	else if (_v == 7)
	{
		if (_h <= 2)
		{
			_h = 2; _v = 6;
		}
		else if (_h >= 6)
		{
			_h = 6; _v = 6;
		}
	}
	if (_v == 8)
	{
		if ((_h == 3) || (_h == 5))
		{
			_h = 4;
		}
		else if (_h == 2)
		{
			_h = 3; _v = 7;
		}
		else if (_h == 6)
		{
			_h = 5; _v = 7;
		}
		else if (_h < 2)
		{
			_h = 2; _v = 6;
		}
		else if (_h > 6)
		{
			_h = 6; _v = 6;
		}
	}
	_H = (EL_PoiAngle_t)_h;
	_V = (EL_PoiAngle_t)_v;
	return;
}

void EL_Ring::LayerPoi_Norm()
{
	int poi_h = (int)Poi.H, poi_v = (int)Poi.V;
	for (int hi = poi_h - 1; hi <= (poi_h + 1); hi++)
		for (int vi = poi_v - 1; vi <= (poi_v + 1); vi++)
		{
			//SetPixel(hi, vi, EL_Gain_t::Off, EL_Gain_t::Full);
			//ResetPixel(hi, vi);
			ResetPixel(hi, vi);
		}

	//ResetPixel(poi_h, poi_v);
	if ((Poi.H == SetPoi.H) && (Poi.V == SetPoi.V))
		return;
	else
	{
		if (Poi.H < SetPoi.H)
			poi_h++;
		else if (Poi.H > SetPoi.H)
			poi_h--;

		if (Poi.V < SetPoi.V)
			poi_v++;
		else if (Poi.V > SetPoi.V)
			poi_v--;
		Poi.H = (EL_PoiAngle_t)poi_h;
		Poi.V = (EL_PoiAngle_t)poi_v;
	}
}

void EL_Ring::LayerLid_Norm()
{
	switch (State)
	{
	case EL_State_t::Closed:
		switch (Task)
		{
		case EL_Action_t::None:
			ResetAllPixels();
			break;
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_A;
			LidPosition_A();
			break;
		case EL_Action_t::Close:
			ResetAllPixels();
			Task = EL_Action_t::None;			
			break;
		case EL_Action_t::Wink:
			Task = EL_Action_t::Open;
			State = EL_State_t::LidPosition_A; 
			LidPosition_A();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_A:
		switch (Task)
		{		
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_B;
			LidPosition_B();
			break;
		case EL_Action_t::Close:
			State = EL_State_t::Closed;
			ResetAllPixels();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::Closed;
			ResetAllPixels();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_B:
		switch (Task)
		{
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_C;
			LidPosition_C();
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_A;
			LidPosition_A();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_A;
			LidPosition_A();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_C:
		switch (Task)
		{
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_D;
			LidPosition_D();
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_B;
			LidPosition_B();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_B;
			LidPosition_B();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_D:
		switch (Task)
		{
		case EL_Action_t::Open:
			State = EL_State_t::Open;
			LidPosition_Open();
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_C;
			LidPosition_C();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_C;
			LidPosition_C();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_E:
		switch (Task)
		{
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_F;
			LidPosition_F();
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_D;
			LidPosition_D();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_D;
			LidPosition_D();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_F:
		switch (Task)
		{
		case EL_Action_t::Open:
			State = EL_State_t::Open;			
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_E;
			LidPosition_E();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_E;
			LidPosition_E();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::Open:
		switch (Task)
		{
		case EL_Action_t::None:
			LidPosition_Open();
			break;
		case EL_Action_t::Open:
			Task = EL_Action_t::None;
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_F;
			LidPosition_F();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_F;
			LidPosition_F();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	default:
		return;
		break;
	}
}

void EL_Ring::LayerLid_Narrow()
{
	switch (State)
	{
	case EL_State_t::Closed:
		switch (Task)
		{
		case EL_Action_t::None:
			ResetAllPixels();
			break;
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_A;
			LidPosition_A();
			break;
		case EL_Action_t::Close:
			Task = EL_Action_t::None;
			ResetAllPixels();
			break;
		case EL_Action_t::Wink:
			Task = EL_Action_t::Open;
			State = EL_State_t::LidPosition_A;
			LidPosition_A();			
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_A:
		switch (Task)
		{
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_B;
			LidPosition_B();
			break;
		case EL_Action_t::Close:
			State = EL_State_t::Closed;
			ResetAllPixels();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::Closed;
			ResetAllPixels();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_B:
		switch (Task)
		{
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_C;
			LidPosition_C();
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_A;
			LidPosition_A();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_A;
			LidPosition_A();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_C:
		switch (Task)
		{
		case EL_Action_t::Open:
			State = EL_State_t::LidPosition_D;
			LidPosition_D();
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_B;
			LidPosition_B();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_B;
			LidPosition_B();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	case EL_State_t::LidPosition_D:
		switch (Task)
		{
		case EL_Action_t::Open:
			Task = EL_Action_t::None;
			State = EL_State_t::Open;
			LidPosition_Open();			
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_C;
			LidPosition_C();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_C;
			LidPosition_C();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;	
	case EL_State_t::Open:
		switch (Task)
		{
		case EL_Action_t::None:
			LidPosition_Open();
			break;
		case EL_Action_t::Open:
			Task = EL_Action_t::None;
			break;
		case EL_Action_t::Close:
			State = EL_State_t::LidPosition_D;
			LidPosition_D();
			break;
		case EL_Action_t::Wink:
			State = EL_State_t::LidPosition_D;
			LidPosition_D();
			break;
		default:
			Task = EL_Action_t::None;
			break;
		}
		return;
		break;
	default:
		return;
		break;
	}
}

void EL_Ring::TaskManager()
{
	if ((micros() - refresh_timer_time) < 35000)return; // 35ms

	refresh_timer_time = micros();

	switch (DispStyle)
	{
	case EL_DisplayStyle_t::Norm:
		// Turn all On.
		SetAllPixels();
		// Apply Pupil masks
		LayerPoi_Norm();
		// Apply Eyelid masks
		LayerLid_Norm();
		return;
		break;
	case EL_DisplayStyle_t::Narrow:		
		// Turn all On.
		SetAllPixels();
		// Apply Pupil masks
		LayerPoi_Norm();
		// Apply Eyelid masks
		LayerLid_Narrow();
		return;
		break;
	default:
		return;
		break;
	}
	return;	
}

EL_Status_t EL_Ring::UpdatePixels()
{
	EL_Status_t ReturnStatus = EL_Status_t::Good;
	int _h; int _v;
	for (int pixel_index = 0; pixel_index < 8; pixel_index++)
	{
		_h = (int)PixelArray[pixel_index]->H;
		_v = (int)PixelArray[pixel_index]->V;

		if (DisplayData[_h][_v].RedGain != PixelArray[pixel_index]->RedGain)
		{
			PixelArray[pixel_index]->RedGain = DisplayData[_h][_v].RedGain;
			PixelArray[pixel_index]->Status = EL_Status_t::Update;
			ReturnStatus = EL_Status_t::Update;
		}
		if (DisplayData[_h][_v].WhiteGain != PixelArray[pixel_index]->WhiteGain)
		{
			PixelArray[pixel_index]->WhiteGain = DisplayData[_h][_v].WhiteGain;
			PixelArray[pixel_index]->Status = EL_Status_t::Update;
			ReturnStatus = EL_Status_t::Update;
		}		
	}
	return ReturnStatus;
}

W_Ret_t EL_Ring::Write()
{
	int return_value = 0;
	EL_Status_t NewData = UpdatePixels();

	if (NewData == EL_Status_t::Good)return W_Ret_t::W_Ok;
	if (COut == W_COut_t::W_All)
	{
		printf("Writing ");
		if (Position == EL_Position_t::Left)
			printf("Left ");
		else
			printf("Right ");
		printf("PCA9685 device - ");
	}

	for (int pixel_index = 0; pixel_index < 8; pixel_index++)
	{
		if (PixelArray[pixel_index]->Status == EL_Status_t::Update)
		{
			return_value = wiringPiI2CWriteReg16(fd_i2c_dev, (int)PixelArray[pixel_index]->Red_Register, (int)PixelArray[pixel_index]->RedGain);
			return_value = wiringPiI2CWriteReg16(fd_i2c_dev, (int)PixelArray[pixel_index]->White_Register, (int)PixelArray[pixel_index]->WhiteGain);
		}
		if (return_value == 0)
		{
			PixelArray[pixel_index]->Status = EL_Status_t::Good;
			if (COut == W_COut_t::W_All)
				printf("*");
		}			
		else
		{
			if (COut != W_COut_t::W_Silent)
			{
				printf("Failed to write to ");
				if (Position == EL_Position_t::Left)
					printf("Left ");
				else
					printf("Right ");
				printf("PCA9685 device.\n Bus busy? %s\n", strerror(errno));
			}
		}		
	}
	if (COut == W_COut_t::W_All)		
		printf(" - Done.\n");
	if (return_value == 0)
		return W_Ret_t::W_Ok;
	return W_Ret_t::W_Busy;
}

W_Ret_t EL_Ring::Action(EL_Action_t _Task)
{
	if (Task == EL_Action_t::None)
	{
		Task = _Task;
		return W_Ret_t::W_Ok;
	}
	return W_Ret_t::W_Busy;
}

EL_Action_t EL_Ring::Action()
{
	return Task;
}

void EL_Ring::Set_Poi(EL_PoiAngle_t _Poih, EL_PoiAngle_t _Poiv)
{
	ClampPoi(_Poih, _Poiv);
	SetPoi.H = _Poih;
	SetPoi.V = _Poiv;
	return;
}

void EL_Ring::Set_Poi(EL_Poi_c _Poi)
{
	ClampPoi(_Poi.H, Poi.V);
	SetPoi = _Poi;
	return;
}

void EL_Ring::Set_Gain(EL_Gain_t _Gain)
{
	if (_Gain == SetGain)
		return;
	SetGain = _Gain;
	return;
}

void EL_Ring::Set_Colour(EL_Colours_t _Colour)
{
	SetColour = _Colour;
	return;
}

void EL_Ring::Set_DisplayStyle(EL_DisplayStyle_t _Disp)
{
	if (_Disp == DispStyle) return;
	DispStyle = _Disp;
	return;
}

W_Ret_t EL_Ring::Update()
{
	switch (Status)
	{
	//case EL_Status_t::Broken:
	//	//Status = EL_Status_t::Init;
	//	//return W_Ret_t::W_Err;
	//	break;
	case EL_Status_t::Init:
		Status = InitDriver();
		break;
	default:
		TaskManager();
		return Write();
		break;
	}
	return W_Ret_t::W_Ok;
}



