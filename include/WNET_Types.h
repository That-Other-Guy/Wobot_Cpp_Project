#pragma once

#ifndef WNET_TYPES_H_
#define WNET_TYPES_H_

#include <string>
#include <cstdint>

class WNET_Envelope_c
{
public:
		int fd_return_address;
		std::string contents;
};

#endif // !WNET_TYPES_H_
