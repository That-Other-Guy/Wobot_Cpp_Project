#include <cstdio>
#include <unistd.h>
#include "SM_StateMachine.h"

SM_StateMachine::SM_StateMachine(W_COut_t _COut)
{	
	Expressions = new XPR_Engine(W_COut_t::W_Silent);
	Emote = XPR_Emote_t::Contentment;
}

SM_StateMachine::~SM_StateMachine()
{
	Expressions->~XPR_Engine();
	delete Expressions;
}

W_Ret_t SM_StateMachine::Go()
{
	printf("Run...\n");
	while (true)
	{
			usleep(100);
			Expressions->Update(Emote, EL_Gain_t::Med);		
	}
	return W_Ret_t();
}
