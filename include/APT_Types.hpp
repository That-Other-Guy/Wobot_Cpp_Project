#pragma once

#ifndef APT_TYPES_H_
#define APT_TYPES_H_

enum class APT_Status_t
{
	Idle,
	DataRequest,
	DataWait,
	Processing,
	NextTask,
};

enum class APT_State_t
{
	Initialize,
	Request_UncompTemp,
	Request_UncompPress,
	ProcessResults,
};

enum class APT_DataStatus_t
{
	Unavailable,
	Available,
};

typedef struct
{
	short OSS;
	short AC1;
	short AC2;
	short AC3;
	unsigned short AC4;
	unsigned short AC5;
	unsigned short AC6;

	long X1;
	long X2;
	long X3;

	short MB;
	short MC;
	short MD;

	short B1;
	short B2;
	long B3;
	unsigned long B4;
	long B5;
	long B6;
	unsigned long B7;

	long UT;
	long UP;

	long T;
	long P;
}APT_Data_s;

#endif // !APT_TYPES_H_

