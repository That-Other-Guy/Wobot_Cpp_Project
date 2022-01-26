#pragma once

#ifndef EL_TYPES_H_
#define EL_TYPES_H_

#include <stdint.h>

enum class EL_Status_t // : uint8_t
{
	Broken = -1,
	Good = 0,
	Update = 1,
	Init,
};

enum class EL_RingPosH_t :int
{
	A = 4,
	B = 6,
	C = 8,
	D = 6,
	E = 4,
	F = 2,
	G = 0,
	H = 2
};

enum class EL_RingPosV_t : int
{
	A = 0, 
	B = 2, 
	C = 4, 
	D = 6, 
	E = 8, 
	F = 6, 
	G = 4, 
	H = 2
};

enum class EL_Gain_t : uint16_t
{
	Off = 0x0000,
	Min = 0x000Fu,
	Low = 0x003Fu,
	Med = 0x00FFu,
	High = 0x03FFu,
	Full = 0x0FFFu,
};

class EL_PixColour_c
{
public:
	EL_Gain_t WhiteGain;
	EL_Gain_t RedGain;
};

enum class EL_Colours_t // : uint8_t
{
	Custom = 0,
	White = 0x1u,
	Red = 0x2u,
	Pink = 0x3u,
};

enum class EL_Position_t // : uint8_t
{
	Left,
	Right,
};

enum class EL_PoiAngle_t : int
{
	Pos_60 = 0,
	Pos_45 = 1,
	Pos_30 = 2,
	Pos_15 = 3,
	Centre = 4,
	Neg_15 = 5,
	Neg_30 = 6,
	Neg_45 = 7,
	Neg_60 = 8,
};

class EL_Poi_c
{
public:
	EL_PoiAngle_t V;
	EL_PoiAngle_t H;
};

// Stlye of display / animation.

enum class EL_DisplayStyle_t
{
	Norm,
	Narrow,
};

// State of selected animation.

enum class EL_State_t // : uint8_t
{
	Closed = 0x00u,
	LidPosition_A = 0x01u,
	LidPosition_B = 0x02u,
	LidPosition_C = 0x03u,
	LidPosition_D = 0x04u,
	LidPosition_E = 0x05u,
	LidPosition_F = 0x06u,
	Open = 0x07u,
};

// Action / animation.

enum class EL_Action_t
{
	None = 0x0u,
	Open = 0x1u,
	Close = 0x2u,
	Wink = 0x4u,
};



#endif // !EL_TYPES_H_

