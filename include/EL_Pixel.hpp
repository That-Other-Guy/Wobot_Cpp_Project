#pragma once

#include "EL_Types.hpp"
#include "PCA9685.hpp"
#ifndef EL_PIXEL_H_
#define EL_PIXEL_H_

class EL_Pixel
{
public:
	EL_Status_t Status;
	// Gain
	EL_Gain_t RedGain;
	EL_Gain_t WhiteGain;
	// On Low Register, 12 bit regiser, Auto Increment.
	PCA9685_RegAddr_t White_Register;
	PCA9685_RegAddr_t Red_Register;
	// Grid position on display.
	EL_RingPosH_t H;
	EL_RingPosV_t V;
	// Gains for Red and White
};

#endif // !EL_PIXEL_H_