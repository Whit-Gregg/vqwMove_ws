/****************************************************************************
*
*   Copyright (c) 2006 Dave Hylands     <dhylands@gmail.com>
*
*   This program is free software; you can redistribute it and/or modify
*   it under the terms of the GNU General Public License version 2 as
*   published by the Free Software Foundation.
*
*   Alternatively, this software may be distributed under the terms of BSD
*   license.
*
*   See README and COPYING for more details.
*
****************************************************************************/
/**
*
*   @file   Crc8.c 
*
*   @brief  This file contains the definition of the CRC-8 algorithim
*           used by SMBus
*           
*
*****************************************************************************/

/* ---- Include Files ----------------------------------------------------- */

//#include "Config.h"

#include "CRC8.h"

//#include "Log.h"

/* ---- Public Variables -------------------------------------------------- */
/* ---- Private Constants and Types --------------------------------------- */
/* ---- Private Variables ------------------------------------------------- */
/* ---- Private Function Prototypes --------------------------------------- */
/* ---- Functions --------------------------------------------------------- */

/****************************************************************************/
/**
*   Calculates the CRC-8 used as part of SMBus.
*
*   CRC-8 is defined to be x^8 + x^2 + x + 1
*
*   To use this function use the following template:
*
*       crc = Crc8( crc, data );
*/

#if 0   // Traditional implementation

#define POLYNOMIAL    (0x1070U << 3) 

unsigned char Crc8( unsigned char inCrc, unsigned char inData )
{
	int i;
    unsigned short  data;

    data = inCrc ^ inData;
    data <<= 8;
  
	for ( i = 0; i < 8; i++ ) 
    {
		if (( data & 0x8000 ) != 0 )
        {
            data = data ^ POLYNOMIAL;
        }
		data = data << 1;
	}

#if 0
#if defined( LogBuf2 )
    LogBuf2( "Crc8: data:0x%02x crc:0x%02x\n", inData, (unsigned char)( data >> 8 ));
#else
    Log( "Crc8: data:0x%02x crc:0x%02x\n", inData, (unsigned char)( data >> 8 ));
#endif
#endif

	return (unsigned char)( data >> 8 );

} // Crc8

// //#else   // Optimized for 8 bit CPUs (0x22 bytes on ATMega128 versus 0x30 for above version)

unsigned char Crc8( unsigned char inCrc, unsigned char inData )
{
	unsigned char   i;
    unsigned char   data;

    data = inCrc ^ inData;
  
	for ( i = 0; i < 8; i++ ) 
    {
        if (( data & 0x80 ) != 0 )
        {
            data <<= 1;
            data ^= 0x07;
        }
        else
        {
            data <<= 1;
        }
	}

#if 0
#if defined( LogBuf2 )
    LogBuf2( "Crc8: data:0x%02x crc:0x%02x\n", inData, data );
#else
    Log( "Crc8: data:0x%02x crc:0x%02x\n", inData, data );
#endif
#endif
	return data;

} // Crc8

#endif


static const uint8_t crc8_table[256] = {
	0,   7,  14,   9,  28,  27,  18,  21,  56,  63,  54,  49,  36,  35,  42,  45,
	112, 119, 126, 121, 108, 107,  98, 101,  72,  79,  70,  65,  84,  83,  90,  93,
	224, 231, 238, 233, 252, 251, 242, 245, 216, 223, 214, 209, 196, 195, 202, 205,
	144, 151, 158, 153, 140, 139, 130, 133, 168, 175, 166, 161, 180, 179, 186, 189,
	199, 192, 201, 206, 219, 220, 213, 210, 255, 248, 241, 246, 227, 228, 237, 234,
	183, 176, 185, 190, 171, 172, 165, 162, 143, 136, 129, 134, 147, 148, 157, 154,
	39,  32,  41,  46,  59,  60,  53,  50,  31,  24,  17,  22,   3,   4,  13,  10,
	87,  80,  89,  94,  75,  76,  69,  66, 111, 104,  97, 102, 115, 116, 125, 122,
	137, 142, 135, 128, 149, 146, 155, 156, 177, 182, 191, 184, 173, 170, 163, 164,
	249, 254, 247, 240, 229, 226, 235, 236, 193, 198, 207, 200, 221, 218, 211, 212,
	105, 110, 103,  96, 117, 114, 123, 124,  81,  86,  95,  88,  77,  74,  67,  68,
	25,  30,  23,  16,   5,   2,  11,  12,  33,  38,  47,  40,  61,  58,  51,  52,
	78,  73,  64,  71,  82,  85,  92,  91, 118, 113, 120, 127, 106, 109, 100,  99,
	62,  57,  48,  55,  34,  37,  44,  43,   6,   1,   8,  15,  26,  29,  20,  19,
	174, 169, 160, 167, 178, 181, 188, 187, 150, 145, 152, 159, 138, 141, 132, 131,
	222, 217, 208, 215, 194, 197, 204, 203, 230, 225, 232, 239, 250, 253, 244, 243,
};

uint8_t Crc8(uint8_t old_crc, uint8_t new_byte)
{
	uint8_t new_crc = crc8_table[old_crc ^ new_byte];
	return new_crc;
}

#if defined( CFG_CRC8BLOCK )

uint8_t Crc8Block(uint8_t old_crc, uint8_t *pBuf, uint8_t bufSize)
{
	uint8_t data;
	uint8_t remainder = old_crc;
	int x;
	for (x = 0; x< bufSize; x++) {
		data = pBuf[x];
		remainder = crc8_table[remainder ^ data];
	}
	return remainder;
}

#endif




#if (0)
/****************************************************************************/
/**
*   Calculates the CRC-8 used as part of SMBus over a block of memory.
*/

uint8_t Crc8Block( uint8_t crc, uint8_t *data, uint8_t len )
{
    while ( len > 0 )
    {
        crc = Crc8( crc, *data++ );
        len--;
    }

    return crc;

} // Crc8Block

#endif  // CFG_CRC8BLOCK

/** @} */

