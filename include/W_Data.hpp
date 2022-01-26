#pragma once

#ifndef W_DATA_H_
#define W_DATA_H_

#include "W_Types.hpp"
#include "EL_Types.hpp"

class W_Data
{
public:
	struct
	{
		W_COut_t COut_Setting;
		EL_Gain_t RL_Max_Gain;
	}Setup;
};

#endif // !W_DATA_H_