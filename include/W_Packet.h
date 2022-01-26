#pragma once

#ifndef W_PACKET_H_
#define W_PACKET_H_

#include "W_MsgId.h"
#include "W_Types.hpp"

class W_Packet_c
{
public:
	W_MsgID_t ID;
	W_DLC_t DLC;
	uint8_t Byte[16];
};

#endif // !W_PACKET_H_