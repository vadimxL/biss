/*
 * main.c
 */


#include "BiSS_RT.def"
#include "limits.h"
#include "Extrn_Asm.var"

#define FPGA_MFB_INTERFACE_ADD                   (0x00004100)
#define FPGA_MFB_TX_BUFFER_ADD                   (0x00004480)

volatile Uint16 Cntr_3125 = 1;
Uint32 AX0_u32_Cumul_TO_Err_Cntr;
Uint32 AX0_u32_Cumul_Crc_Err_Cntr;

extern Uint16 AX0_CRRNT_8_VAR_PAGE;
extern Uint16 AX0_CRRNT_9_VAR_PAGE;
extern void FillCRCTable(unsigned int u16_crc_polynom, int drive, int enc_source, int fdbk_dest);
extern void foo_asm(Uint16* data);

//extern void BISSC_COMMUNICATION_FEEDBACK();

Uint16 InitVars(void);

int main(void)
{
	void (*BissRtFdbk)(void) = (void (*)(void))(Uint32)(&BISSC_COMMUNICATION_FEEDBACK);
	Uint16 loop = 2;
	Uint16 cntr = 0;
	Uint16 comm_fdbk_flags = (COMM_FDBK_READY_MASK | ABS_POSITION_REQ_SENT_MASK | COMM_FDBK_INIT_ABS_POS_MASK);
	Uint64 foo_data = 0;

	foo_data = 0xCAFEBADDBEEFDEAD;
	*(volatile Uint64*)(AX0_u32_FPGA_Tx_Rx_Regs_Ptr) = 0xFFFFF86C8870135A;
	AX0_u16_Parsing_Regs_Ptr = ((Uint16)&AX0_CRRNT_8_VAR_PAGE >> 6);
	foo_asm((Uint16*)&foo_data);
	AX0_u16_Parsing_Regs_Ptr = ((Uint16)&AX0_CRRNT_9_VAR_PAGE >> 6);
	foo_asm((Uint16*)&foo_data);

	AX0_u16_Comm_Fdbk_Flags_A_1 = comm_fdbk_flags;
	InitVars();

	FillCRCTable(0x43, 0, 0, 0);

	while(loop--)
	{
	   BissRtFdbk();
	   cntr++;
	}

	AX0_u16_Comm_Fdbk_Flags_A_1 = comm_fdbk_flags;

	loop = 2;
	while(loop)
	{
		BissCommunicationFeedback();
		cntr++;
	}

	return 0;
}

extern Uint32 AX0_u32_Biss_Single_Turn_Eff_Mask;
extern Uint32 AX0_u32_Biss_Multi_Turn_Eff_Mask;
Uint16 InitVars(void)
{
   Uint16 tmp;
	AX0_u32_BiSSC_MT_ST_Length_Resolution = (12ul << 24) | (9ul << 16) | (19ul << 8) | (13ul << 0);
	tmp = AX0_u32_BiSSC_MT_ST_Length_Resolution & 0xFF;
   AX0_u32_Biss_Single_Turn_Eff_Mask = ((1ul << tmp) - 1);
   tmp = (AX0_u32_BiSSC_MT_ST_Length_Resolution >> 16) & 0xFF;
   AX0_u32_Biss_Multi_Turn_Eff_Mask = ((1ul << tmp) - 1);
   tmp = (AX0_u32_BiSSC_MT_ST_Length_Resolution >> 8) & 0xFF;
	AX0_u32_BiSSC_MT_ST_Length = (AX0_u32_BiSSC_MT_ST_Length_Resolution >> 24) + tmp;
	AX0_u32_FPGA_Base_Regs_Ptr = FPGA_MFB_INTERFACE_ADD;
	AX0_u32_FPGA_Tx_Rx_Regs_Ptr = FPGA_MFB_TX_BUFFER_ADD;

	*(volatile Uint16*)(AX0_u32_FPGA_Base_Regs_Ptr + 0xb) = 2;
	//*(volatile Uint64*)(AX0_u32_FPGA_Tx_Rx_Regs_Ptr + 0x40) = 0xFFFFF86C8870135A;
	*(volatile Uint64*)(AX0_u32_FPGA_Tx_Rx_Regs_Ptr) = 0xFFFFF86C8870135A;

	AX0_u16_Parsing_Regs_Ptr = ((Uint16)&AX0_CRRNT_8_VAR_PAGE >> 6);
	return 12;
}
