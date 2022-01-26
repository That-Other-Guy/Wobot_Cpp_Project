#pragma once

#ifndef _XPR_NODE_HPP_
#define _XPR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/u_int32.hpp"

#include "XPR_Engine.hpp"

class XPR_Node : public rclcpp::Node
{
private:
	rclcpp::TimerBase::SharedPtr main_loop_timer;
	rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr xpr_subscription;
	XPR_Engine * XPR;
public:
	XPR_Node();
	~XPR_Node();

private:
	void LoopCallback_MainLoop();
	void TopicCallback_XPR(const std_msgs::msg::UInt32::SharedPtr _Msg) const;
};



#endif /* _XPR_NODE_HPP_ */