/*

*/
#include <cerrno>
#include <cstring>
#include <clocale>
#include <thread>
#include <chrono>
#include <cstdio>
#include <signal.h>
#include <unistd.h>

#include <functional>
#include <memory>
#include <string>

#include "XPR_Node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

XPR_Node::XPR_Node()
	:
	Node("XPR_Node")
{
	XPR = new XPR_Engine(W_COut_t::W_Norm);
	XPR->pGain = XPR_Gain_t::Med;
	XPR->pEmote = XPR_Emote_t::Sleep;
	xpr_subscription = this->create_subscription<std_msgs::msg::UInt32>("expression", 10, std::bind(&XPR_Node::TopicCallback_XPR, this, _1));
	main_loop_timer = this->create_wall_timer(500us, std::bind(&XPR_Node::LoopCallback_MainLoop, this));
	return;
}
XPR_Node::~XPR_Node()
{
	delete XPR;
	return;
}

void XPR_Node::LoopCallback_MainLoop()
{
	XPR->Update(XPR->pEmote, XPR->pGain, XPR->pFocus);
	return;
}
void XPR_Node::TopicCallback_XPR(const std_msgs::msg::UInt32::SharedPtr _Msg) const
{
	XPR_Msg_s Msg;
	memcpy(&Msg, &_Msg->data, sizeof(Msg));
	RCLCPP_INFO(this->get_logger(), "Rec' Mood Req - Mood: %d - Focus: %d - Gain: %d\n[ %d:%d:%d:%d:%d:%d:%d:%d ]\n",
		(uint8_t)Msg.Emote, (uint8_t)Msg.Focus, (uint8_t)Msg.Gain,
		(uint8_t)Msg.A0, (uint8_t)Msg.A1, (uint8_t)Msg.A2, (uint8_t)Msg.A3,
		(uint8_t)Msg.B0, (uint8_t)Msg.B1, (uint8_t)Msg.B2, (uint8_t)Msg.B3);
	XPR->pEmote = Msg.Emote;
	XPR->pFocus = Msg.Focus;
	XPR->pGain = Msg.Gain;
	return;
}

int main(int _argc, char* _argv[])
{
	rclcpp::init(_argc, _argv);
	rclcpp::spin(std::make_shared<XPR_Node>());
	rclcpp::shutdown();
	return 0;
}
