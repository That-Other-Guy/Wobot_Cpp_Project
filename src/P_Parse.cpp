#include "P_Parse.h"
#include <sstream>

P_Parse::P_Parse(W_COut_t _COut)
{
	COut = _COut;
}

P_Parse::~P_Parse()
{
}

//W_Ret_t Communications_Engine::PacketToString(Wobot_Packet& _Packet, std::string& _str)
//{
//	std::string new_string {};
//	new_string.append("S,");
//
//	char data_byte[10];
//	sprintf(data_byte, "%x", (uint16_t)_Packet.ID);
//	new_string.append(data_byte);
//	new_string.append(",");
//
//	for (uint8_t i = 0; i < (uint8_t)_Packet.DLC; i++)
//	{
//		sprintf(data_byte, "%x", (uint8_t)_Packet.Byte[i]);
//		new_string.append(data_byte);
//		new_string.append(",");
//	}
//	new_string.append("E");
//
//	return W_Ret_t::W_Ok;
//}
//
//W_Ret_t Communications_Engine::StringToPacket(std::string& _str, Wobot_Packet& _Packet)
//{
//	//Rules. Comma seperation.
//	//Packet Format
//	//'P,0x200,0xFF,0xEE,E\n'
//
//	// First seperate by '\n'
//
//	std::vector<std::string> str_chunks;
//	std::stringstream stringstream(_str);
//
//	while (stringstream.good())	
//	{		
//
//		std::string substring;
//		getline(stringstream, substring, ',');
//		str_chunks.push_back(substring);
//	}
//	
//	std::vector<std::string>::iterator chunks_it = str_chunks.begin();
//	std::vector<std::string>::iterator vect_S;
//	std::vector<std::string>::iterator vect_E;
//
//	bool S_present = false;
//	bool E_present = false;
//
//	while (true)
//	{
//		std::string substring = *chunks_it;
//		std::string::iterator substring_it = substring.begin();
//
//		if ( *substring_it == 'S' )
//		{
//			if (S_present == true)
//				return W_Ret_t::W_Invalid;
//			++substring_it;
//			if (*substring_it != NULL)
//				return W_Ret_t::W_Invalid;
//			vect_S = chunks_it;
//			S_present = true;
//		}
//		else if ( *substring_it == 'E' )
//		{
//			if (S_present == false)
//				return W_Ret_t::W_Invalid;
//			if (E_present == true)
//				return W_Ret_t::W_Invalid;
//			vect_E = chunks_it;
//			E_present = true;
//		}
//		else if (*substring_it == '0')
//		{
//			if (S_present == false)
//				return W_Ret_t::W_Invalid;
//			++substring_it;
//			if ( !((*substring_it != 'x') || (*substring_it != 'X')))
//				return W_Ret_t::W_Invalid;
//			++substring_it;
//			int hexadecimal_figures = 0;
//			while (true)
//			{
//				if (*substring_it == NULL)
//				{
//					if (hexadecimal_figures == 0)
//						return W_Ret_t::W_Invalid;
//					if (substring_it == substring.end())
//						break;
//					return W_Ret_t::W_Invalid;
//				}
//				if ( !((*substring_it >= '0') && (*substring_it <= '9')))
//					if ( !((*substring_it >= 'A') && (*substring_it <= 'F')) )
//						if ( !((*substring_it >= 'a') && (*substring_it <= 'f')))
//							return W_Ret_t::W_Invalid;	
//				hexadecimal_figures++;
//				++substring_it;
//			}
//		}
//		else
//		{
//			return W_Ret_t::W_Invalid;
//		}	
//		++chunks_it;
//		if (chunks_it == str_chunks.end())
//			break;
//	}
//
//	if ( !(S_present && E_present) )
//		return W_Ret_t::W_Invalid;
//
//	chunks_it = vect_S;
//	++chunks_it;
//
//	_Packet.ID = (Wobot_Message_ID)std::stoi(*chunks_it, nullptr, 0);
//
//	++chunks_it;
//	uint8_t dlc = 0;
//
//	while (true)
//	{
//		_Packet.Byte[dlc] = (uint8_t)std::stoi(*chunks_it, nullptr, 0);
//		++dlc;
//		if(dlc > (uint8_t)W_DLC::_8_Bytes)
//			return W_Ret_t::W_Invalid;
//		++chunks_it;		
//		if (chunks_it == vect_E)break;
//	}
//	_Packet.DLC = (W_DLC)(dlc);	
//	return W_Ret_t::W_Ok;
//}

P_StrClass_t P_Parse::ParseString(WNET_Envelope_c _Mail)
{	
	Mail.fd_return_address = _Mail.fd_return_address;
	Mail.contents = "Cool message bro!\n";

	Queue = P_StrClass_t::WNET_Envelope;

	switch (Queue)
	{
	case P_StrClass_t::Nonsense:
		return P_StrClass_t::Nonsense;
		break;
	case P_StrClass_t::Empty:
		return P_StrClass_t::Empty;
		break;
	case P_StrClass_t::W_Packet:
		return P_StrClass_t::W_Packet;
		break;
	case P_StrClass_t::WNET_Envelope:
		return P_StrClass_t::WNET_Envelope;
		break;
	case P_StrClass_t::OS_Exec:
		return P_StrClass_t::OS_Exec;
		break;
	default:
		return P_StrClass_t::Nonsense;
		break;
	}	
}

W_Packet_c P_Parse::Get_Packet()
{
	Queue = P_StrClass_t::Empty;
	return Packet;
}

WNET_Envelope_c P_Parse::Get_Mail()
{
	Queue = P_StrClass_t::Empty;
	return Mail;
}


