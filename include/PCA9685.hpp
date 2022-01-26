#ifndef PCA9685_H_
#define PCA9685_H_

#include <cstdint>

enum class PCA9685_Bit_t : uint8_t
{
	Unset = 0x0u,
	Set = 0x1u,
};

enum class PCA9685_DevAddr_t : uint8_t
{
	LeftDevice = 0x40u,
	RightDevice = 0x42u,
};

enum class PCA9685_RegAddr_t : uint8_t
{
	MODE1 = 0x00u,
	MODE2 = 0x01u,
	SUBADR1 = 0x02u,
	SUBADR2 = 0x03u,
	SUBADR3 = 0x04u,
	ALLCALLADR = 0x05u,

	LED0_ON_L = 0x06u,
	LED0_ON_H = 0x07u,
	LED0_OFF_L = 0x08u,
	LED0_OFF_H = 0x09u,

	LED1_ON_L = 0x0Au,
	LED1_ON_H = 0x0Bu,
	LED1_OFF_L = 0x0Cu,
	LED1_OFF_H = 0x0Du,

	LED2_ON_L = 0x0Eu,
	LED2_ON_H = 0x0Fu,
	LED2_OFF_L = 0x10u,
	LED2_OFF_H = 0x11u,

	LED3_ON_L = 0x12u,
	LED3_ON_H = 0x13u,
	LED3_OFF_L = 0x14u,
	LED3_OFF_H = 0x15u,

	LED4_ON_L = 0x16u,
	LED4_ON_H = 0x17u,
	LED4_OFF_L = 0x18u,
	LED4_OFF_H = 0x19u,

	LED5_ON_L = 0x1Au,
	LED5_ON_H = 0x1Bu,
	LED5_OFF_L = 0x1Cu,
	LED5_OFF_H = 0x1Du,

	LED6_ON_L = 0x1Eu,
	LED6_ON_H = 0x1Fu,
	LED6_OFF_L = 0x20u,
	LED6_OFF_H = 0x21u,

	LED7_ON_L = 0x22u,
	LED7_ON_H = 0x23u,
	LED7_OFF_L = 0x24u,
	LED7_OFF_H = 0x25u,

	LED8_ON_L = 0x26u,
	LED8_ON_H = 0x27u,
	LED8_OFF_L = 0x28u,
	LED8_OFF_H = 0x29u,

	LED9_ON_L = 0x2Au,
	LED9_ON_H = 0x2Bu,
	LED9_OFF_L = 0x2Cu,
	LED9_OFF_H = 0x2Du,

	LEDA_ON_L = 0x2Eu,
	LEDA_ON_H = 0x2Fu,
	LEDA_OFF_L = 0x30u,
	LEDA_OFF_H = 0x31u,

	LEDB_ON_L = 0x32u,
	LEDB_ON_H = 0x33u,
	LEDB_OFF_L = 0x34u,
	LEDB_OFF_H = 0x35u,

	LEDC_ON_L = 0x36u,
	LEDC_ON_H = 0x37u,
	LEDC_OFF_L = 0x38u,
	LEDC_OFF_H = 0x39u,

	LEDD_ON_L = 0x3Au,
	LEDD_ON_H = 0x3Bu,
	LEDD_OFF_L = 0x3Cu,
	LEDD_OFF_H = 0x3Du,

	LEDE_ON_L = 0x3Eu,
	LEDE_ON_H = 0x3Fu,
	LEDE_OFF_L = 0x40u,
	LEDE_OFF_H = 0x41u,

	LEDF_ON_L = 0x42u,
	LEDF_ON_H = 0x43u,
	LEDF_OFF_L = 0x44u,
	LEDF_OFF_H = 0x45u,

	ALL_LED_ON_L = 0xFAu,
	ALL_LED_ON_H = 0xFBu,
	ALL_LED_OFF_L = 0xFCu,
	ALL_LED_OFF_H = 0xFDu,

	PRE_SCALE = 0xFEu,
	TESTMODE = 0xFFu
};

struct PCA9685_Mode_s
{
	union
	{
		struct
		{
			PCA9685_Bit_t ALLCALL : 1;   //PCA9685_MODE1_ // Respond to all call address.
			PCA9685_Bit_t SUB3 : 1;	    //PCA9685_MODE1_
			PCA9685_Bit_t SUB2 : 1;	    //PCA9685_MODE1_			
			PCA9685_Bit_t SUB1 : 1;	    //PCA9685_MODE1_
			PCA9685_Bit_t SLEEP : 1;     //PCA9685_MODE1_ // Low power mode
			PCA9685_Bit_t AI : 1;        //PCA9685_MODE1_ // Auto increment register address
			PCA9685_Bit_t EXTCLK : 1;    //PCA9685_MODE1_
			PCA9685_Bit_t RESTART : 1;   //PCA9685_MODE1_
										//
			PCA9685_Bit_t OUTNE0 : 1;    //PCA9685_MODE2_ // Respond to all call address.
			PCA9685_Bit_t OUTNE1 : 1;    //PCA9685_MODE2_
			PCA9685_Bit_t OUTDRV : 1;    //PCA9685_MODE2_
			PCA9685_Bit_t OCH : 1;	    //PCA9685_MODE2_
			PCA9685_Bit_t INVRT : 1;     //PCA9685_MODE2_ // Low power mode
			PCA9685_Bit_t Reserved5 : 1; //PCA9685_MODE2_ // Auto increment register address
			PCA9685_Bit_t Reserved6 : 1; //PCA9685_MODE2_
			PCA9685_Bit_t Reserved7 : 1; //PCA9685_MODE2_
		};
		uint16_t raw;
	};
};

#endif // !PCA9685_H_
