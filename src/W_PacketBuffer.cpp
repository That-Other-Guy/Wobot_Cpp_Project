#include "W_PacketBuffer.h"

W_PacketBuffer::W_PacketBuffer()
{
	read_pos = 0;
	write_pos = 0;
}

W_PacketBuffer::~W_PacketBuffer()
{
}

W_Ret_t W_PacketBuffer::WriteBuffer(W_Packet_c _Packet)
{
	Ring[write_pos] = _Packet;
	write_pos++;
	if (write_pos >= size)
		write_pos = 0;
	if (write_pos == read_pos)
	{
		read_pos++;
		if (read_pos >= size)
			read_pos = 0;
		return W_Ret_t::W_Overflow;
	}
	return W_Ret_t::W_Ok;
}

W_Ret_t W_PacketBuffer::ReadBuffer(W_Packet_c& _Packet)
{
	if (read_pos == write_pos)
	{
		return W_Ret_t::W_NoData;
	}
	_Packet = Ring[read_pos];
	read_pos++;
	if (read_pos >= size)
		read_pos = 0;
	return W_Ret_t::W_Ok;
}
