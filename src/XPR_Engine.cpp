#include "XPR_Engine.hpp"
#include <cstdlib>
#include <time.h>
#include <wiringPi.h>
#include <cstdio>

XPR_Engine::XPR_Engine(W_COut_t _COut) :
	Ring_Left(EL_Position_t::Left, _COut),
	Ring_Right(EL_Position_t::Right, _COut)
{
	COut = _COut;

	if(COut != W_COut_t::W_Silent)// || TerminalSetting != W_COut_t::W_ErrOnly)
		printf("Expressions Engine Start.\n");

	Mood = XPR_Emote_t::Contentment;
	Focus = XPR_Focus_t::Random;

	srand((unsigned int)time(NULL));
		
	thyme_blink_delay = millis();
	thyme_poi_delay = thyme_blink_delay;
}

XPR_Engine::~XPR_Engine()
{
	Ring_Left.~EL_Ring();
	Ring_Right.~EL_Ring();
}

W_Ret_t XPR_Engine::Blink()
{
	if ( (Ring_Left.Action(EL_Action_t::Wink) == W_Ret_t::W_Busy )
		| (Ring_Right.Action(EL_Action_t::Wink) == W_Ret_t::W_Busy) )
		return W_Ret_t::W_Busy;
	return W_Ret_t::W_Ok;
}

W_Ret_t XPR_Engine::PoiSet(EL_Poi_c _Poi)
{
	W_Ret_t Return = W_Ret_t::W_Ok;
	Ring_Left.Set_Poi(_Poi);
	Ring_Right.Set_Poi(_Poi);
	return Return;
}

void XPR_Engine::Contentment()
{
	if ((unsigned long)(millis() - thyme_blink_delay) > (unsigned long)random_blink_time)
	{
		random_blink_time = __XPR_BLINKRAND_CONTENT_;
		thyme_blink_delay = millis();
		Blink();
	}
	EL_Poi_c Poi;
	switch (Focus)
	{
	case XPR_Focus_t::Random:
		if ((unsigned long)(millis() - thyme_poi_delay) > (unsigned long)random_poi_time)
		{
			random_poi_time = __XPR_POIRAND_CONTENT_;
			thyme_poi_delay = millis();
			int _poih = rand() % __EL_DISPLAYSIZE_;
			int _poiv = rand() % __EL_DISPLAYSIZE_;
			Poi.H = (EL_PoiAngle_t)_poih;
			Poi.V = (EL_PoiAngle_t)_poiv;
			PoiSet(Poi);
		}		
		return;
		break;
	case XPR_Focus_t::Up:
		Poi.H = EL_PoiAngle_t::Centre;
		Poi.V = EL_PoiAngle_t::Pos_60;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::LeftUp:
		Poi.H = EL_PoiAngle_t::Pos_45;
		Poi.V = EL_PoiAngle_t::Pos_45;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::Left:
		Poi.H = EL_PoiAngle_t::Pos_60;
		Poi.V = EL_PoiAngle_t::Centre;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::LeftDown:
		Poi.H = EL_PoiAngle_t::Pos_45;
		Poi.V = EL_PoiAngle_t::Neg_45;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::Down:
		Poi.H = EL_PoiAngle_t::Centre;
		Poi.V = EL_PoiAngle_t::Neg_60;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::RightDown:
		Poi.H = EL_PoiAngle_t::Neg_45;
		Poi.V = EL_PoiAngle_t::Neg_45;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::Right:
		Poi.H = EL_PoiAngle_t::Neg_60;
		Poi.V = EL_PoiAngle_t::Centre;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::RightUp:
		Poi.H = EL_PoiAngle_t::Neg_45;
		Poi.V = EL_PoiAngle_t::Pos_45;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::Centre:
		Poi.H = EL_PoiAngle_t::Centre;
		Poi.V = EL_PoiAngle_t::Centre;
		PoiSet(Poi);
		return;
	default:
		return;
		break;
	}
	return;
}

void XPR_Engine::Resentment()
{
	if ((unsigned long)(millis() - thyme_blink_delay) > (unsigned long)random_blink_time)
	{
		random_blink_time = __XPR_BLINKRAND_EXCITE_;
		thyme_blink_delay = millis();
		Blink();
	}

	EL_Poi_c Poi;
	switch (Focus)
	{
	case XPR_Focus_t::Random:
		if ((unsigned long)(millis() - thyme_poi_delay) > (unsigned long)random_poi_time)
		{
			random_poi_time = __XPR_POIRAND_EXCITE_;
			thyme_poi_delay = millis();
			int _poih = rand() % __EL_DISPLAYSIZE_;
			int _poiv = rand() % __EL_DISPLAYSIZE_;
			Poi.H = (EL_PoiAngle_t)_poih;
			Poi.V = (EL_PoiAngle_t)_poiv;
			PoiSet(Poi);
		}
		return;
		break;
	case XPR_Focus_t::Up:
		Poi.H = EL_PoiAngle_t::Centre;
		Poi.V = EL_PoiAngle_t::Pos_60;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::LeftUp:
		Poi.H = EL_PoiAngle_t::Pos_45;
		Poi.V = EL_PoiAngle_t::Pos_45;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::Left:
		Poi.H = EL_PoiAngle_t::Pos_60;
		Poi.V = EL_PoiAngle_t::Centre;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::LeftDown:
		Poi.H = EL_PoiAngle_t::Pos_45;
		Poi.V = EL_PoiAngle_t::Neg_45;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::Down:
		Poi.H = EL_PoiAngle_t::Centre;
		Poi.V = EL_PoiAngle_t::Neg_60;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::RightDown:
		Poi.H = EL_PoiAngle_t::Neg_45;
		Poi.V = EL_PoiAngle_t::Neg_45;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::Right:
		Poi.H = EL_PoiAngle_t::Neg_60;
		Poi.V = EL_PoiAngle_t::Centre;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::RightUp:
		Poi.H = EL_PoiAngle_t::Neg_45;
		Poi.V = EL_PoiAngle_t::Pos_45;
		PoiSet(Poi);
		return;
		break;
	case XPR_Focus_t::Centre:
		Poi.H = EL_PoiAngle_t::Centre;
		Poi.V = EL_PoiAngle_t::Centre;
		PoiSet(Poi);
		return;
	default:
		return;
		break;
	}
	return;
}

W_Ret_t XPR_Engine::ExpressionsManager()
{
	if ((unsigned long)(millis() - thyme_looking_at) > __DEFAULT_RETENTION)
	{
		Focus = XPR_Focus_t::Random;
		Looking = W_Bit_t::Unset;
	}

	switch (Mood)
	{
	case XPR_Emote_t::Contentment:
		Contentment();
		break;
	case XPR_Emote_t::Resentment:
		Contentment();
		break;
	case XPR_Emote_t::Curiosity:
		break;
	case XPR_Emote_t::Excitement:
		break;
	default:
		break;
	}
	return W_Ret_t::W_Ok;
}

void XPR_Engine::SetMood(XPR_Emote_t _Mood)
{
	if (Mood == _Mood)return;
	Mood = _Mood;
	switch (Mood)
	{
	case XPR_Emote_t::Sleep:
		Ring_Left.Action(EL_Action_t::Close);
		Ring_Right.Action(EL_Action_t::Close);
		break;
	case XPR_Emote_t::Contentment:
		Ring_Left.Set_Colour(EL_Colours_t::White);
		Ring_Right.Set_Colour(EL_Colours_t::White);
		Ring_Left.Set_DisplayStyle(EL_DisplayStyle_t::Norm);
		Ring_Right.Set_DisplayStyle(EL_DisplayStyle_t::Norm);
		break;
	case XPR_Emote_t::Resentment:
		Ring_Left.Set_Colour(EL_Colours_t::Red);
		Ring_Right.Set_Colour(EL_Colours_t::Red);
		Ring_Left.Set_DisplayStyle(EL_DisplayStyle_t::Narrow);
		Ring_Right.Set_DisplayStyle(EL_DisplayStyle_t::Narrow);
		break;
	case XPR_Emote_t::Curiosity:
		break;
	case XPR_Emote_t::Excitement:
		Ring_Left.Set_Colour(EL_Colours_t::Pink);
		Ring_Right.Set_Colour(EL_Colours_t::Pink);
		break;
	default:
		break;
	}
	if (COut != W_COut_t::W_Silent && COut != W_COut_t::W_ErrOnly)
	{
		switch (Mood)
		{
		case XPR_Emote_t::Sleep:
			printf("Mood -> Sleep.\n");
			break;
		case XPR_Emote_t::Contentment:
			printf("Mood -> Contentment.\n");
			break;
		case XPR_Emote_t::Resentment:
			printf("Mood -> Resentment.\n");
			break;
		case XPR_Emote_t::Curiosity:
			printf("Mood -> Curiosity.\n");
			break;
		case XPR_Emote_t::Excitement:
			printf("Mood -> Excitement.\n");
			break;
		default:
			printf("Mood -> Unkown.\n");
			break;
		}
	}
	return;
}

void XPR_Engine::Look(XPR_Focus_t _Focus)
{
	Focus = _Focus;
	thyme_looking_at = micros();
	Looking = W_Bit_t::Set;
	return;
}

void XPR_Engine::SetGain(XPR_Gain_t _Gain)
{
	EL_Gain_t NewGain;

	switch (_Gain)
	{
	case XPR_Gain_t::Off:
		NewGain = EL_Gain_t::Off;
		break;
	case XPR_Gain_t::Min:
		NewGain = EL_Gain_t::Min;
		break;
	case XPR_Gain_t::Low:
		NewGain = EL_Gain_t::Low;
		break;
	case XPR_Gain_t::Med:
		NewGain = EL_Gain_t::Med;
		break;
	case XPR_Gain_t::High:
		NewGain = EL_Gain_t::High;
		break;
	case XPR_Gain_t::Full:
		NewGain = EL_Gain_t::Full;
		break;
	default:
		return;
		break;
	}
	Ring_Left.Set_Gain(NewGain);
	Ring_Right.Set_Gain(NewGain);
	return;
}

W_Ret_t XPR_Engine::Update(XPR_Emote_t _Mood, XPR_Gain_t _Gain, XPR_Focus_t _Focus)
{
	SetMood(_Mood);
	Look(_Focus);
	SetGain(_Gain);	

	ExpressionsManager();

	Ring_Left.Update();
	Ring_Right.Update();

	return W_Ret_t::W_Ok;
}




