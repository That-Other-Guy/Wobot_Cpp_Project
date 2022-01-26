#pragma once
#ifndef XPR_TYPES_H_
#define XPR_TYPES_H_

#include "W_Types.hpp"

enum class XPR_Emote_t : uint8_t
{
	Sleep = 0x0u,
	Contentment = 0x1u,
	Resentment = 0x2u,
	Curiosity = 0x3u,
	Excitement = 0x4u,
};

enum class XPR_Focus_t : uint8_t
{
	Random = 0x0u,
	Centre = 0x1u,
	Up = 0x2u,
	LeftUp = 0x3u,
	Left = 0x4u,
	LeftDown = 0x5u,
	Down = 0x6u,
	RightDown = 0x7u,
	Right = 0x8u,
	RightUp = 0x9u,	
};

enum class XPR_Gain_t : uint8_t
{
	Off = 0x0u,
	Min = 0x1u,
	Low = 0x2u,
	Med = 0x3u,
	High = 0x4u,
	Full = 0x5u,
};

typedef struct
{
	XPR_Emote_t Emote;
	XPR_Focus_t Focus;
	XPR_Gain_t Gain;

	W_Bit_t A0 : 1;
	W_Bit_t A1 : 1;
	W_Bit_t A2 : 1;
	W_Bit_t A3 : 1;

	W_Bit_t B0 : 1;
	W_Bit_t B1 : 1;
	W_Bit_t B2 : 1;
	W_Bit_t B3 : 1;
}XPR_Msg_s;

#endif // !XPR_TYPES_H_
