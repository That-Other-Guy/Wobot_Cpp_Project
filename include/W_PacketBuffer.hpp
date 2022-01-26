#pragma once

#ifndef W_PACKETBUFFER_H_
#define W_PACKETBUFFER_H_

#include "W_Packet.h"
#include "W_Types.hpp"

class W_PacketBuffer
{
private:
	static constexpr unsigned int size = 32;
	W_Packet_c Ring[size];
	unsigned int read_pos;
	unsigned int write_pos;
public:
	W_PacketBuffer();
	~W_PacketBuffer();
public:
	W_Ret_t WriteBuffer(W_Packet_c _Packet);
	W_Ret_t ReadBuffer(W_Packet_c& _Packet);
};

#endif // !W_PACKETBUFFER_
