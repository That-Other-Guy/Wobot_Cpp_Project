
#include "State_Node.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


State_Node::State_Node()
	:
	Node("State_Node"),
	exp_count(0)
{
	main_loop_timer = this->create_wall_timer(100us, std::bind(&State_Node::LoopCallback_MainLoop, this));
	xpr_publish_timer = this->create_wall_timer(1s, std::bind(&State_Node::LoopCallback_XprPub, this));

	xpr_publication = this->create_publisher<std_msgs::msg::UInt32>("expression", 10);

	return;
}

State_Node::~State_Node()
{
	return;
}

void State_Node::LoopCallback_MainLoop()
{
	
	return;
}

void State_Node::LoopCallback_XprPub()
{
	if (exp_count >= 20)
	{
		exp_count = 0;
		switch (Emote)
		{
		case XPR_Emote_t::Sleep:
			Emote = XPR_Emote_t::Resentment;
			break;
		case XPR_Emote_t::Contentment:
			Emote = XPR_Emote_t::Sleep;
			break;
		case XPR_Emote_t::Resentment:
			Emote = XPR_Emote_t::Contentment;
			break;
		default:
			break;
		}
		auto message = std_msgs::msg::UInt32();

		XPR_Msg_s Msg;
		memset(&Msg, 0, sizeof(Msg));

		Msg.Emote = Emote;
		Msg.Focus = XPR_Focus_t::Random;
		Msg.Gain = XPR_Gain_t::Med;

		memcpy(&message.data, &Msg, sizeof(Msg));
		xpr_publication->publish(message);

		RCLCPP_INFO(this->get_logger(), "Publishing Mood Req - Mood: %d - Focus: %d - Gain: %d\n[ %d:%d:%d:%d:%d:%d:%d:%d ]\n",
			(uint8_t)Msg.Emote, (uint8_t)Msg.Focus, (uint8_t)Msg.Gain,
			(uint8_t)Msg.A0, (uint8_t)Msg.A1, (uint8_t)Msg.A2, (uint8_t)Msg.A3,
			(uint8_t)Msg.B0, (uint8_t)Msg.B1, (uint8_t)Msg.B2, (uint8_t)Msg.B3);
	}
	exp_count++;
	return;
}

int main(int _argc, char* _argv[])
{
	rclcpp::init(_argc, _argv);
	rclcpp::spin(std::make_shared<State_Node>());
	rclcpp::shutdown();
	return 0;
}