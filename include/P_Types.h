#pragma once

#ifndef P_TYPES_H_
#define P_TYPES_H_

enum class P_StrClass_t
{
	Nonsense = -1,
	Empty = 0,
	W_Packet = 1,
	WNET_Envelope,
	OS_Exec,
};

#endif // !P_TYPES_H_