#pragma once

#ifndef ACC_TYPES_H_
#define ACC_TYPES_H_

enum class ACC_Status_t
{
	Idle,
	DataRequest,
	DataWait,
	Processing,
	NextTask,
};

enum class ACC_State_t
{
	Initialize,
	RequestData,
	LoadData,
	ProcessResults,
};

class ACC_Data_s
{
public:
	float fX;
	float fY;
	float fZ;
};

#endif // !ACC_TYPES_H_