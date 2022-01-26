#pragma once

#ifndef W_MSGID_H_
#define W_MSGID_H_

#include <stdint.h>

enum class W_MsgID_t : uint16_t
{
	nullID = 0x000u,
	midHead_STM32 = 0x110u,
};

#endif // !W_MSGID_H_