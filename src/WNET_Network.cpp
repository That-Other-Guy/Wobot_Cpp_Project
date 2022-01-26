#include "WNET_Network.h"

#include <cerrno>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <sstream>

#include <sys/types.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include <netdb.h>
#include <arpa/inet.h>
#include <string.h>
#include <string>

void* WNET_Network::Net_Thread(void* _arg)
{
	WNET_Network* This = (WNET_Network*)_arg;	

	if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
		printf("Network socket thread start.\n");
	/*
	Step - 1, call os, create the socket
	*/
	int tcp_socket_listner = socket(AF_INET, SOCK_STREAM, 0);
	if (tcp_socket_listner == -1)
	{
		if (This->COut != W_COut_t::W_Silent)
			printf("[W_ERROR] Could not create network socket.\nERRNO: %d\n", errno);
				
		sleep(120);
		if (This->COut != W_COut_t::W_Silent)
			printf("Restarting network thread\n");

		This->thread_is_running = false;
		return nullptr;
	}

	if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
		printf("Network socket created.\n");
	/*
	Step - 2, call os, set socket options using the socket using desciptor
	*/
	int opt_val = 1;

	int opt_return_value = setsockopt(tcp_socket_listner, SOL_SOCKET, SO_REUSEADDR, &opt_val, sizeof(int));
	if (opt_return_value == -1)
	{
		if (tcp_socket_listner == -1)
		{
			if (This->COut != W_COut_t::W_Silent)
				printf("[W_ERROR] Could not set socket options for network socket.\nERRNO: %d\n", errno);

			sleep(120);
			if (This->COut != W_COut_t::W_Silent)
				printf("Restarting network thread\n");

			This->thread_is_running = false;
			return nullptr;
		}
	}
	if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
		printf("Network socket options set.\n");
	/*
	Step - 3, call os, bind the socket using desciptor
	structure describing an Internet socket address. sockaddr_in
	*/
	struct sockaddr_in host_sockaddr;
	host_sockaddr.sin_family = AF_INET;
	host_sockaddr.sin_addr.s_addr = INADDR_ANY;
	host_sockaddr.sin_port = htons(__NETWORK_PORTNO);
	inet_pton(AF_INET, "0.0.0.0", &host_sockaddr.sin_addr);

	int bind_value = bind(tcp_socket_listner, (struct sockaddr*)&host_sockaddr, sizeof(host_sockaddr));
	if (bind_value == -1)
	{
		if (tcp_socket_listner == -1)
		{
			if (This->COut != W_COut_t::W_Silent)
				printf("[W_ERROR] Could not bind to network socket.\nERRNO: %d\n", errno);

			sleep(120);
			if (This->COut != W_COut_t::W_Silent)
				printf("Restarting network thread\n");

			This->thread_is_running = false;
			return nullptr;
		}
	}
	if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
		printf("Network socket bind success.\n");
	/*
	Step - 3, call os, listen to the socket for connection attempts.
	structure describing an Internet socket address. sockaddr_in
	*/
	int listen_return_val = listen(tcp_socket_listner, 10);
	if (listen_return_val == -1)
	{
		if (This->COut != W_COut_t::W_Silent)
			printf("[W_ERROR] Could not listen on network socket.\nERRNO: %d\n", errno);

		sleep(120);
		if (This->COut != W_COut_t::W_Silent)
			printf("Restarting network thread\n");

		This->thread_is_running = false;
		return nullptr;
	}
	if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
		printf("Network socket listen succes.\n");

	fd_set fd_set_master;
	FD_ZERO(&fd_set_master);
	FD_SET(tcp_socket_listner, &fd_set_master);
	int fdmax = tcp_socket_listner;

	struct sockaddr_in client_sockaddr;
	socklen_t remote_addrlen = sizeof(client_sockaddr);

	if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
		printf("Network socket ready. Waiting on client.\n");
	
	for (;;)
	{
		fd_set fd_set_read = fd_set_master;

		int socket_tally = select(fdmax + 1, &fd_set_read, nullptr, nullptr, nullptr);
		//if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
		//	printf("Available files: %d", socket_tally);

		for (int fd_it = 0; fd_it <= fdmax; fd_it++)
		{
			if (FD_ISSET(fd_it, &fd_set_read))
				// Hit
			{
				// if new hit on original fd -> new client
				if (fd_it == tcp_socket_listner)
				{
					int fd_new_client = accept(tcp_socket_listner, (struct sockaddr*)&client_sockaddr, &remote_addrlen);
					if (fd_new_client == -1)
					{
						if (This->COut != W_COut_t::W_Silent)
						{
							printf("[W_ERROR] Could not accept network socket client.\nERRNO: %d\n"
								, errno);
						}
					}
					else
					{
						FD_SET(fd_new_client, &fd_set_master);
						if (fd_new_client > fdmax)fdmax = fd_new_client;

						std::string client_ip = inet_ntoa(client_sockaddr.sin_addr);
						int remote_port = ntohs(client_sockaddr.sin_port);
						if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
							printf("Network socket client connected.\nClient: %d IP: %s PORT: %d\nWaiting for Data\n", fd_new_client, (char*)client_ip.c_str(), remote_port);

						std::string client_welcome = "[W_MSG] Connection Accepted.\n";
						send(fd_new_client, client_welcome.c_str(), client_welcome.size() + 1, 0);
					}
				}
				// if new hit on existing fd. -> data rdy
				else
				{
					char data_buffer[__BUFFER_SIZE];
					memset(data_buffer, 0, __BUFFER_SIZE);
					memset(&client_sockaddr, 0, remote_addrlen);
					getsockname(fd_it, (struct sockaddr*)&client_sockaddr, &remote_addrlen);
					std::string client_ip = inet_ntoa(client_sockaddr.sin_addr);
					int remote_port = ntohs(client_sockaddr.sin_port);

					long int return_value = recv(fd_it, data_buffer, __BUFFER_SIZE, 0);
					if (return_value <= 0)
					{
						if (return_value == -1)
						{
							if (This->COut != W_COut_t::W_Silent)
								printf("[W_ERROR] Could not read network socket client.\nERRNO: %d\nDropping Client!\n"
									, errno);
						}
						if (return_value == 0)
						{
							if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
								printf("Client disconnected.\n");
						}
						if (!(This->COut == W_COut_t::W_Silent || This->COut == W_COut_t::W_ErrOnly))
							printf("Client: %d IP: %s PORT: %d\n", fd_it, (char*)client_ip.c_str(), remote_port);

						close(fd_it);
						FD_CLR(fd_it, &fd_set_master);
					}
					else
					{
						WNET_Envelope_c rxMail;
						rxMail.fd_return_address = fd_it;
						rxMail.contents = data_buffer;

						pthread_mutex_lock(&This->mutex_recv_buffer);
						auto it = This->RecvBuffer.begin();
						This->RecvBuffer.emplace(it, rxMail);
						pthread_mutex_unlock(&This->mutex_recv_buffer);

						if (This->COut == W_COut_t::W_All)
							printf("Network client data recieved.\n Client message: \'%s\'\n", data_buffer);
					}
				}
			}
		}		
	}
	This->thread_is_running = false;
	return nullptr;
}

W_Ret_t WNET_Network::Send()
{ 
	pthread_mutex_lock(&mutex_send_buffer);
	if (!SendBuffer.empty())
	{
		auto it = SendBuffer.end();
		--it;
		WNET_Envelope_c txMail = *it;
		long int return_value = send(txMail.fd_return_address, txMail.contents.c_str(), txMail.contents.size() + 1, 0);
		if (return_value == -1)
		{
			if (COut != W_COut_t::W_Silent)
				printf("[W_ERROR] Could not send to client.\nERRNO: %d.\n"
					, errno);
			pthread_mutex_unlock(&mutex_send_buffer);
			return W_Ret_t::W_Err;
		}
		SendBuffer.pop_back();
		if (COut == W_COut_t::W_All)
			printf("Send success.\n Client message: \'%s\'\n", txMail.contents.c_str());
	}
	pthread_mutex_unlock(&mutex_send_buffer);
	return W_Ret_t::W_Ok;
}

W_Ret_t WNET_Network::ReStart()
{
	if (thread_is_running == false)
	{
		thread_is_running = true;
		SendBuffer.clear();
		RecvBuffer.clear();
		pthread_Net_Thread = new pthread_t;
		int return_value = pthread_create(pthread_Net_Thread, NULL, Net_Thread, (void*)this);
		if (return_value != 0)
		{
			if (COut != W_COut_t::W_Silent)
				printf("[W_ERROR] Networking thread could not be created.\nERRNO: %d\n", return_value);
			exit(0);
		}
	}
	return W_Ret_t::W_Ok;
}

WNET_Network::WNET_Network(W_COut_t _COut)
{
	COut = _COut;
	thread_is_running = false;
	pthread_mutex_init(&mutex_recv_buffer, NULL);
	pthread_mutex_init(&mutex_send_buffer, NULL);
	return;
}

WNET_Network::~WNET_Network()
{

}

W_Ret_t WNET_Network::Update()
{
	ReStart();
	pthread_mutex_lock(&mutex_recv_buffer);
	if (!RecvBuffer.empty())
	{
		pthread_mutex_unlock(&mutex_recv_buffer);
		return W_Ret_t::W_GotMail;
	}
	pthread_mutex_unlock(&mutex_recv_buffer);
	return Send();
}

W_Ret_t WNET_Network::Post(WNET_Envelope_c _post)
{
	pthread_mutex_lock(&mutex_send_buffer);
	auto it = SendBuffer.begin();
	SendBuffer.emplace(it, _post);
	pthread_mutex_unlock(&mutex_send_buffer);
	return W_Ret_t::W_Ok;
}

WNET_Envelope_c WNET_Network::GetNext()
{
	pthread_mutex_lock(&mutex_recv_buffer);
	auto it = RecvBuffer.end();
	--it;
	WNET_Envelope_c rxMail = *it;
	RecvBuffer.pop_back();
	pthread_mutex_unlock(&mutex_recv_buffer);
	return rxMail;
	
}


//void* Communications_Engine::DomainSocket_Thread(void* _arg) {}
///*
//{
//	Communications_Engine * This = (Communications_Engine*)_arg;
//	pthread_mutex_lock(&This->mutex_threadstart);
//
//	if (! (This->TerminalSetting == W_Out_t::W_Silent || This->TerminalSetting == W_Out_t::W_ErrOnly) )
//		printf("Domain socket thread start.\n");
//		
//	struct sockaddr_un name;
//	char data_buffer[__BUFFER_SIZE];
//	int connection_socket = 0;
//	int return_value = 0;
//	int fd_data_socket = 0;	
//
//	unlink(__DOMAIN_SOCKET_NAME);
//
//	connection_socket = socket(AF_UNIX, SOCK_STREAM, 0);
//	
//	if (connection_socket == -1)
//	{
//		if (This->TerminalSetting != W_Out_t::W_Silent)
//		{
//			if (This->TerminalSetting != W_Out_t::W_Silent)
//				printf("[W_ERROR] Could not create domain socket.\nERRNO: %d\n", errno);
//			pthread_mutex_unlock(&This->mutex_threadstart);
//			sleep(120);
//			if (This->TerminalSetting != W_Out_t::W_Silent)
//				printf("Restarting domain socket thread\n");
//			This->DomainSocket_Thread_is_running = false;
//			return nullptr;
//		}
//	}
//	if (!(This->TerminalSetting == W_Out_t::W_Silent || This->TerminalSetting == W_Out_t::W_ErrOnly))
//		printf("Domain socket created.\n");
//	
//	//Init.
//
//	memset(&name, 0, sizeof(struct sockaddr_un));
//	
//	name.sun_family = AF_UNIX;
//	strncpy(name.sun_path, __DOMAIN_SOCKET_NAME, sizeof(name.sun_path) - 1);
//	
//	for (;;)
//	{
//		int trys = 0;
//		for (;;)
//		{
//			return_value = bind(connection_socket, (const struct sockaddr*)&name, sizeof(struct sockaddr_un));
//
//			if (return_value == -1)
//			{
//				trys++;
//				if (This->TerminalSetting != W_Out_t::W_Silent)
//					printf("[W_ERROR] Could not bind to domain socket.\nERRNO: %d\n", errno);
//
//				pthread_mutex_unlock(&This->mutex_threadstart);
//				if (trys >= __TRYS)
//				{
//					if (This->TerminalSetting != W_Out_t::W_Silent)
//						printf("Restarting domain socket thread\n");
//					This->DomainSocket_Thread_is_running = false;
//					return nullptr;
//				}
//				sleep(15);
//				pthread_mutex_lock(&This->mutex_threadstart);
//				if (This->TerminalSetting != W_Out_t::W_Silent)
//					printf("Retrying 'bind domain socket'. Retry attempt #%d\n", trys);
//			}
//			else
//			{
//				if (!(This->TerminalSetting == W_Out_t::W_Silent || This->TerminalSetting == W_Out_t::W_ErrOnly))
//					printf("Domain socket bind success.\n");
//				break;
//			}
//		}
//
//		trys = 0;
//		for (;;)
//		{
//			return_value = listen(connection_socket, 20);
//			if (return_value == -1)
//			{
//				trys++;
//				if (This->TerminalSetting != W_Out_t::W_Silent)
//					printf("[W_ERROR] Could not listen on domain socket.\nERRNO: %d\n", errno);
//
//				pthread_mutex_unlock(&This->mutex_threadstart);
//				if (trys >= __TRYS)
//				{
//					if (This->TerminalSetting != W_Out_t::W_Silent)
//						printf("Restarting domain socket thread\n");
//					This->DomainSocket_Thread_is_running = false;
//					return nullptr;
//				}
//				sleep(15);
//				pthread_mutex_lock(&This->mutex_threadstart);
//				if (This->TerminalSetting != W_Out_t::W_Silent)
//					printf("Retrying 'listen domain socket'. Retry attempt #%d\n", trys);
//			}
//			else
//			{
//				if (!(This->TerminalSetting == W_Out_t::W_Silent || This->TerminalSetting == W_Out_t::W_ErrOnly))
//					printf("Domain socket listen succes.\n");
//				break;
//			}
//		}
//		pthread_mutex_unlock(&This->mutex_threadstart);
//
//
//		int trys_accept = 0;
//		int trys_recv = 0;
//		int number_of_clients = 0;
//
//		for (;;)
//		{
//			if (!(This->TerminalSetting == W_Out_t::W_Silent || This->TerminalSetting == W_Out_t::W_ErrOnly))
//				printf("Domain socket ready. Waiting on client.\n");
//
//			fd_data_socket = accept(connection_socket, NULL, NULL);
//
//			if (fd_data_socket == -1)
//			{
//				trys_accept++;
//				if (This->TerminalSetting != W_Out_t::W_Silent)
//				{
//					printf("[W_ERROR] Could not accept network socket client.\nERRNO: %d\nFailed to accept %d total clients.\n"
//						, errno, trys_accept);
//				}
//			}
//			number_of_clients++;
//			if (!(This->TerminalSetting == W_Out_t::W_Silent || This->TerminalSetting == W_Out_t::W_ErrOnly))
//				printf("Domain socket client connected.\nClient: %d\nWaiting for Data->\n"
//					, number_of_clients);
//
//			for (;;)
//			{
//				char data_buffer[__BUFFER_SIZE];
//				memset(data_buffer, 0, __BUFFER_SIZE);
//
//				return_value = read(fd_data_socket, data_buffer, __BUFFER_SIZE);
//				if (return_value == -1)
//				{
//					trys_recv++;
//					if (This->TerminalSetting != W_Out_t::W_Silent)
//					{
//						printf("[W_ERROR] Could not read domain socket client.\nERRNO: %d\nFailed to read %d total messages.\n"
//							, errno, trys_recv);
//					}
//				}
//				//if (return_value == 0)
//				//{
//				//	if (This->TerminalSetting != W_Out_t::W_Silent)
//				//	{
//				//		if (!(This->TerminalSetting == W_Out_t::W_Silent || This->TerminalSetting == W_Out_t::W_ErrOnly))
//				//			printf("Domain socket client disconnected.\n");
//				//		break;
//				//	}
//				//}
//				if (!(This->TerminalSetting == W_Out_t::W_Silent || This->TerminalSetting == W_Out_t::W_ErrOnly))
//					printf("Domain client data recieved, thank you.\n Client message: \"%s\"\n", data_buffer);
//				std::string data_string = data_buffer;
//				Wobot_Packet New_Packet;
//				W_Ret_t Ret = This->StringToPacket(data_string, New_Packet);
//			}
//		}
//	}
//	return nullptr;