#pragma once

#ifndef P_Parse_H_
#define P_PARSE_H_

#include "P_Types.h"
#include "WNET_Types.h"
#include "W_Packet.h"

class P_Parse
{
private:
	W_COut_t COut;
private:
	P_StrClass_t Queue;
	W_Packet_c Packet;
	WNET_Envelope_c Mail;
public:
	P_Parse(W_COut_t _COut);
	~P_Parse();
public:
	P_StrClass_t ParseString(WNET_Envelope_c _Mail);
	W_Packet_c Get_Packet();
	WNET_Envelope_c Get_Mail();
};

#endif // !P_Parse_H_


