#pragma once

#ifndef W_TYPES_H_
#define W_TYPES_H_
#include <cstdint>

enum class W_Bit_t : uint8_t
{
	Unset = 0x0u,
	Set = 0x1u,
};

enum class W_Ret_t
{
	W_Err = -1,
	W_Ok = 0,
	W_Busy,
	W_Redundant,
	W_NewData,
	W_NoData,
	W_Overflow,
	W_Invalid,
	W_GotMail,
};

enum class W_COut_t : uint8_t
{
	W_Silent = 0x0u,
	W_Norm = 0x1u,
	W_ErrOnly = 0x2u,
	W_All = 0x3u,
};

enum class W_Task_t
{
	Error = -1,
	Idle = 0,
};

enum class W_DLC_t : uint8_t
{
	_0_Bytes = 0,
	_1_Bytes = 1,
	_2_Bytes = 2,
	_3_Bytes = 3,
	_4_Bytes = 4,
	_5_Bytes = 5,
	_6_Bytes = 6,
	_7_Bytes = 7,
	_8_Bytes = 8,
	_9_Bytes = 9,
	_10_Bytes = 10,
	_11_Bytes = 11,
	_12_Bytes = 12,
	_13_Bytes = 13,
	_14_Bytes = 14,
	_15_Bytes = 15,
	_16_Bytes = 16,
};

#endif // !W_TYPES_H_
