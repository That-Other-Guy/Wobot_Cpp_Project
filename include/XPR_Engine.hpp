#ifndef _EXPRESSIONS_HPP_
#define _EXPRESSIONS_HPP_

#include "EL_Ring.hpp"
#include "W_Types.hpp"
#include "XPR_Types.hpp"

#define __XPR_POIRAND_CONTENT_ (rand()%500)+3500
#define __XPR_BLINKRAND_CONTENT_ (rand()%10000)+10000

#define __XPR_POIRAND_EXCITE_ (rand()%250)+1250
#define __XPR_BLINKRAND_EXCITE_ (rand()%1500)+4500

#define __DEFAULT_RETENTION 10000000 // usec

class XPR_Engine
{
private:
	W_COut_t COut;
private:
	XPR_Emote_t Mood;
public:
	XPR_Emote_t pEmote;
	XPR_Gain_t pGain;
	XPR_Focus_t pFocus;
private:
	EL_Ring Ring_Left;
	EL_Ring Ring_Right;
private:
	XPR_Focus_t Focus;
	unsigned long thyme_looking_at;
	int looking_time;

	W_Bit_t Looking;
private:
	unsigned long thyme_blink_delay;
	unsigned long thyme_poi_delay;
	int random_blink_time;
	int random_poi_time;	
public:
	XPR_Engine(W_COut_t _COut);
	~XPR_Engine();
private:
	W_Ret_t Blink();
	W_Ret_t PoiSet(EL_Poi_c _Poi);
private:
	void Contentment();
	void Resentment();
	//void Curiosity();
	//void Excitement();
private:
	W_Ret_t ExpressionsManager();
	void SetMood(XPR_Emote_t _Mood);
	void Look(XPR_Focus_t _Focus);
	void SetGain(XPR_Gain_t _Gain);
public:	
	W_Ret_t Update(XPR_Emote_t _Mood, XPR_Gain_t _Gain, XPR_Focus_t _Focus);	
};

#endif /* _EXPRESSIONS_HPP_ */
