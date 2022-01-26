#ifndef EL_RING_H_
#define EL_RING_H_

#include "PCA9685.hpp"
#include "W_Types.hpp"
#include "EL_Types.hpp"
#include "EL_Pixel.hpp"

#define __EL_DISPLAYSIZE_ 9

class EL_Ring
{
private:
	EL_Position_t Position;
	W_COut_t COut;
	EL_Status_t Status;
private:
	EL_Pixel Pixel_A;
	EL_Pixel Pixel_B;
	EL_Pixel Pixel_C;
	EL_Pixel Pixel_D;
	EL_Pixel Pixel_E;
	EL_Pixel Pixel_F;
	EL_Pixel Pixel_G;
	EL_Pixel Pixel_H;
	EL_Pixel * PixelArray[8];
private:
	int fd_i2c_dev;
private:
	EL_DisplayStyle_t DispStyle;
	EL_State_t State;
	EL_Action_t Task;
	EL_Poi_c SetPoi;
	EL_Poi_c Poi;
private:
	unsigned long refresh_timer_time;
	//int animation_countdown;
private:
	EL_Colours_t SetColour;
	EL_Gain_t SetGain;
	EL_PixColour_c DisplayData[__EL_DISPLAYSIZE_][__EL_DISPLAYSIZE_];
public:
	EL_Ring(EL_Position_t _Pos, W_COut_t _COut);
	~EL_Ring();
private:
	EL_Status_t InitDriver();
private:
	void SetPixel(int _h, int _v, EL_Gain_t _r, EL_Gain_t _w);
	void SetPixel(int _h, int _v);
	void ResetPixel(int _h, int _v);	
	void ResetAllPixels();
	void SetAllPixels();
	void SetAllPixels(EL_Gain_t _Gain); // uses SetColour
	void SetAllPixels(EL_Colours_t _Colour); // uses SetGain
	void SetAllPixels(EL_Colours_t _Colour, EL_Gain_t _Gain);		
	void DimPixel(int _h, int _v);
	void DDimPixel(int _h, int _v);
private:
	void LidPosition_Open();
	void LidPosition_A();
	void LidPosition_B();
	void LidPosition_C();
	void LidPosition_D();
	void LidPosition_E();
	void LidPosition_F();	
private:
	void ClampPoi(EL_PoiAngle_t& _H, EL_PoiAngle_t& _V);
	void LayerPoi_Norm();

	void LayerLid_Norm();
	void LayerLid_Narrow();
private:
	void TaskManager();
	EL_Status_t UpdatePixels();
	W_Ret_t Write();
public:
	W_Ret_t Action(EL_Action_t _Task);
	EL_Action_t Action();
	void Set_Poi(EL_PoiAngle_t  _Poih, EL_PoiAngle_t  _Poiv);
	void Set_Poi(EL_Poi_c  _Poi);
	void Set_Gain(EL_Gain_t _Gain);
	void Set_Colour(EL_Colours_t _Colour);
	void Set_DisplayStyle(EL_DisplayStyle_t _Disp);
	W_Ret_t Update();
};

#endif /* EL_RING_H_ */

