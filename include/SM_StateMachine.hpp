#pragma once
#ifndef SM_STATEMACHINE_H_
#define SM_STATEMACHINE_H_

#include "W_Types.hpp"
#include "XPR_Engine.hpp"

//#include "rclcpp/rclcpp.hpp"
//#include "std_msgs/msg/string.hpp"

class SM_StateMachine
{
private:
	XPR_Engine * Expressions;
	XPR_Emote_t Emote;
public:
	SM_StateMachine(W_COut_t _COut);
	~SM_StateMachine();
public:
	W_Ret_t Go();
};

#endif // !SM_STATEMACHINE_H_





