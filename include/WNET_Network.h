#pragma once

#ifndef WNET_NETWORK_H_
#define _WNET_NETWORK_H_

#include "W_Types.h"
#include "WNET_Types.h"
#include <vector>
#include <pthread.h>
#include <mutex>

#define __BUFFER_SIZE 1024
#define __NETWORK_PORTNO 54000

class WNET_Network
{
private:
	W_COut_t COut;
private:
	int total_connected_clients;
private:
	pthread_t* pthread_Net_Thread;
	static void* Net_Thread(void* _arg);
	bool thread_is_running;
private:
	pthread_mutex_t mutex_send_buffer;
	std::vector<WNET_Envelope_c> SendBuffer;
	pthread_mutex_t mutex_recv_buffer;
	std::vector<WNET_Envelope_c> RecvBuffer;
private:
	W_Ret_t Send();
	W_Ret_t ReStart();
public:
	WNET_Network(W_COut_t _COut);
	~WNET_Network();
	W_Ret_t Update();	
	W_Ret_t Post(WNET_Envelope_c _post);
	WNET_Envelope_c GetNext();
};

#endif // !_WNET_NETWORK_H_