#pragma once

#ifndef _STATE_NODE_HPP_
#define _STATE_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "XPR_Types.hpp"

class State_Node : public rclcpp::Node
{
private:
	XPR_Emote_t Emote;
	XPR_Focus_t Focus;
	int exp_count;	
private:
	rclcpp::TimerBase::SharedPtr main_loop_timer;
	rclcpp::TimerBase::SharedPtr xpr_publish_timer;
	rclcpp::Publisher<std_msgs::msg::UInt32>::SharedPtr xpr_publication;
public:
	State_Node();
	~State_Node();
private:
	void LoopCallback_MainLoop();
	void LoopCallback_XprPub();
};

#endif /* _STATE_NODE_HPP_ */