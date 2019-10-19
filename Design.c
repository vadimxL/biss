/*
 * Design.c
 *
 *  Created on: Oct 9, 2019
 *      Author: vadim.malinovsky
 */

#include "Design.def"
#include "Drive.var"
#include "ModCntrl.def"
#include "MultiAxis.def"
#include "Extrn_Asm.var"



unsigned int  u16_Crc_Table[2][256];


// Gets as a parameter the CRC polynomial and creats a 256 look up table to be used
// in the CRC calculation in RT
void FillCRCTable(unsigned int u16_crc_polynom, int drive, int enc_source, int fdbk_dest)
{
   FDBK_OFF;
   unsigned int *crc_array_ptr = u16_Crc_Table[enc_source];
   unsigned long i, bit, u32_crc_value, u32_crc_mask = 0x0000FFFF, u32_effective_bits = 1L;
   unsigned long u32_top_bit = 0x000000008000, u32_crc_polynom_shifted = 0L;
   REFERENCE_TO_DRIVE;

   for (i = 0; i < 16; i++)
   {
      if (u16_crc_polynom & u32_top_bit) break;
      u32_effective_bits++;
      u32_top_bit >>= 1L;
      u32_crc_mask >>= 1L;
   }
   u32_crc_mask >>= 1L;
   u32_effective_bits = 16L - u32_effective_bits;
   u32_top_bit = 1L << (7L + u32_effective_bits);
   u32_crc_polynom_shifted = ((unsigned long)u16_crc_polynom & 0x00000000FFFFFFFF) << 8L;

   FDBKVAR(AX0_u16_Abs_Crc_Shl) = 8 - (int)u32_effective_bits;

   for (i = 0L; i < 256L; i++)
   {
      u32_crc_value = i << u32_effective_bits;
      for (bit = 8L; bit > 0L; bit--)
      {
         if (u32_crc_value & u32_top_bit)
            u32_crc_value = (u32_crc_value << 1L) ^ u32_crc_polynom_shifted;
         else
            u32_crc_value = (u32_crc_value << 1L);
      }
      *(crc_array_ptr++) = (unsigned int)((u32_crc_value >> 8) & u32_crc_mask);
   }
}
