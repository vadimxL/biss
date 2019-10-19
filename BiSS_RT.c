/*
 * Commutation.c
 *
 *  Created on: 25 September 2019
 *      Author: V. M.
 */

#include "Extrn_Asm.var"
#include "Drive.var"
#include "limits.h"
#include "BiSS_RT.def"
#include "BiSS_C.def"
//#include "Design.def"

BissRegs biss_regs = { (BissRegs1*)&LVAR(AX0_u32_BiSSC_ResponseCmdFrame),
				           (BissRegs2*)&VAR(AX0_u16_BiSSC_RTMBX_Control)     };

#pragma CODE_SECTION(BissAbsPosParse, "biss_rt");
void BissAbsPosParse(Uint64 data, ParsingRegs* regs, Uint16 cds)
{
	// data is:
    //  -----------  -------- --------
    // | "00.....0" | MT POS | ST POS |
    //  ------------ -------- --------

    Uint16 st_width = __byte( (int16*)&biss_regs.regs1->MT_ST_Len_Res, 0 );
    int32 delta;
    Uint32 temp;


    // extract the _Multi-Turn position data
//    regs->Comm_Abs_Pos_Hi = data >> st_width; // Extract MT Position bits;
    int32 mt = data >> st_width; // Extract MT Position bits;

    temp = sizeof(Uint32) * CHAR_BIT - st_width; // remove the MT position
    data = ((Uint32)data << temp) >> temp; // extract Single Turn Position bits

    // if ST resolution is higher than 30-bits, reduce the resolution,
    // by two bits
//    biss_regs.regs1->Abs_Pos_Raw = data; // restore single turn position
    if( st_width > 30 ) data >>= 2;

    // Init position reading
    if ( regs->Comm_Fdbk_Flags_A_1 & COMM_FDBK_INIT_ABS_POS_MASK )
    {
        // clear for next time
        regs->Comm_Fdbk_Flags_A_1 &= ~COMM_FDBK_INIT_ABS_POS_MASK;


        // store the data for CALCULATE_POSITION_FDBK
//        regs->Pos_Fdbk_Hi = regs->Comm_Abs_Pos_Hi; // init pos_fdbk_hi
        regs->Pos_Fdbk_Hi = mt; // init pos_fdbk_hi
        regs->Comm_Abs_Pos_Lo = data;
        regs->Fdbk_Accu = data;

        regs->Comm_Abs_Pos_Half_Delta = 0;
        CALCULATE_POSITION_FDBK(regs->Comm_Abs_Pos_Half_Delta);
    }
    else // regular position reading
    {
        // read biss c control channel
        if ( !(biss_regs.regs2->RTMBX_Control & BISSC_RTMBX_CNTRL_CH_RX_STATE_MASK) ||
                (biss_regs.regs2->RTMBX_Control & BISSC_RTMBX_CNTRL_CH_START_BIT_EVAL_MASK)  )
        { // collect CDS response
            biss_regs.regs1->Response_Cmd_Frame <<= 1;
            __or( (int16*)&biss_regs.regs1->Response_Cmd_Frame , cds );

            // Check if we are finished processing control channel request
            if( biss_regs.regs2->CDM_Cntr > BISSC_CNTRL_CH_REQUEST_BITS_TO_FETCH)
            {
                __and( (int16*)&biss_regs.regs2->RTMBX_Control,
                		~(BISSC_RTMBX_CNTRL_CH_TX_STATE_MASK | BISSC_RTMBX_CNTRL_CH_RX_STATE_MASK));
                __or( (int16*)&biss_regs.regs2->RTMBX_Control, BISSC_RTMBX_RSP_READY_MASK );
            }
        }

        // no cds response, delta = pos - prev_pos
        delta = (int32)((Uint32)data - regs->Comm_Abs_Pos_Lo);
        temp = (delta < 0) ? -delta : delta;

        // Handle roll over
        if( temp > regs->Counts_Per_Rev_Half )
        {
            if ( delta > 0 ) delta -= regs->Counts_Per_Rev; // Negative roll over
            else delta += regs->Counts_Per_Rev; // Positive roll over
        }


        // Aging
        regs->Comm_Abs_Pos_Lo = (Uint32)data;

        delta *= -1;

        //  Avoid calculating new Half_Delta if recovering from any Interpolation Condition
        if( 0 == (regs->Comm_Fdbk_Flags_A_1 & COMM_FDBK_ALL_ERROR_MASK) )
        {
            regs->Comm_Abs_Pos_Half_Delta = delta >> 2;
        }

        delta -= regs->Comm_Abs_Pos_Half_Delta;

        temp = regs->Crc_Consec_Err_Cntr + regs->Time_Out_Consec_Err_Cntr;
        if( temp )
        {
            // Double the TO + CRC error counters to subtract half_deltas for each error
            temp <<= 1;
            // Subtract correction for previous CRC,TO Errors
            delta -= ( (Uint32)temp * regs->Comm_Abs_Pos_Half_Delta );
            regs->Crc_Consec_Err_Cntr = 0; // Zero CRC Consecutive Error Counter
            regs->Time_Out_Consec_Err_Cntr = 0; // Zero Time-Out Consecutive Error Counter
            regs->Comm_Fdbk_Flags_A_1 &= ~(COMM_FDBK_CRC_ERROR_1_MASK | COMM_FDBK_TIME_OUT_ERROR_1_MASK);
        }

        CALCULATE_POSITION_FDBK(delta);
    }

    regs->Comm_Fdbk_Flags_A_1 &= ~COMM_FDBK_INIT_ABS_POS_MASK;
}


#pragma CODE_SECTION(BissSetCdmData, "biss_rt");
void BissSetCdmData( volatile Uint16 *fpga_reg_ptr ) // Transmits the actual payload on the CDM
{
   if ( __tbit( biss_regs.regs1->Reg_Cmd_Frame, 0 ) ) *fpga_reg_ptr = 0x1;
   else  *fpga_reg_ptr = 0x0;
   biss_regs.regs1->Reg_Cmd_Frame >>= 1; // Shift request data for next cycle
   biss_regs.regs2->CDM_Cntr++;
   biss_regs.regs2->RTMBX_Control |= BISSC_RTMBX_CNTRL_CH_RX_STATE_MASK;
}



#pragma CODE_SECTION(BissCtrlChannel, "biss_rt");
void BissCtrlChannel(Uint16 cds)
{
   Uint16 temp;
   volatile Uint16 *fpga_reg_ptr;

   // BG set the request meta data, process in RT
   if( biss_regs.regs2->RTMBX_Control & BISSC_RTMBX_REQ_READY_MASK )
   {
      biss_regs.regs2->RTMBX_Control &= ~BISSC_RTMBX_REQ_READY_MASK; // clear the flag for next time
      // Set the state to process Biss Tx
      biss_regs.regs2->RTMBX_Control |= BISSC_RTMBX_CNTRL_CH_TX_STATE_MASK;
      biss_regs.regs2->CDM_Cntr = 0; // zero the CMD counter
   }

   // State to Transmit command data master (CDM) requests from BG.
   if( biss_regs.regs2->RTMBX_Control & BISSC_RTMBX_CNTRL_CH_TX_STATE_MASK )
   {
	   temp = __byte( (int16*)&biss_regs.regs1->MT_ST_Len_Res, 3 );
	   fpga_reg_ptr = (volatile Uint16*)(AX0_u32_FPGA_Base_Regs_Ptr + 0x3c);

	   // CDM sending
	   if( biss_regs.regs2->CDM_Cntr < BISSC_CNTRL_CH_REQUEST_BITS_TO_SKIP )
	   {   // skip 14 * 62.5uSec each time
		   __inc( (int16*)&biss_regs.regs2->CDM_Cntr );
		   // write 14 "0" to CDM
		   *fpga_reg_ptr = 0;
	   }
	   else if( (biss_regs.regs2->CDM_Cntr == BISSC_CNTRL_CH_REQUEST_BITS_TO_EVAL_START) )
	   {
		   biss_regs.regs2->RTMBX_Control |= BISSC_RTMBX_CNTRL_CH_START_BIT_EVAL_MASK;
		   if( temp ) *fpga_reg_ptr |= 0x2; // Signal to FPGA to start bit evaluation
		   else *fpga_reg_ptr = 0x1; // Set CDM Direct
		   BissSetCdmData(fpga_reg_ptr);
	   }

	   if( ( biss_regs.regs2->RTMBX_Control & BISSC_RTMBX_CNTRL_CH_START_BIT_EVAL_MASK) && cds )
	   {
		   // Check if we are on the start bit evaluation state
		   if( temp ) *fpga_reg_ptr &= ~0x2; // // Signal to FPGA to stop bit evaluation
		   // No stop bit evaluation
		   else biss_regs.regs2->RTMBX_Control &= ~BISSC_RTMBX_CNTRL_CH_START_BIT_EVAL_MASK;
		   BissSetCdmData(fpga_reg_ptr);
	   }
   }
}


#pragma CODE_SECTION(BissReadCommFdbkPos, "biss_rt");
void BissReadCommFdbkPos(ParsingRegs* regs)
{
    Uint64 data;
    Uint16 st_mt_width = biss_regs.regs1->MT_ST_Len, tmp, cds;
    volatile Uint16 *fpga_reg_ptr;

	// poll the data ready bit in FPGA
	fpga_reg_ptr = (volatile Uint16*)(AX0_u32_FPGA_Base_Regs_Ptr + 0xb);
	if( !__tbit( *fpga_reg_ptr , 1 ) ) // data is not ready, check for timeout
	{
		if ( regs->Time_Out_Consec_Err_Cntr < ALLOWED_CONSEC_TIME_OUT_ERRORS )
		{
			regs->Time_Out_Consec_Err_Cntr++;
			regs->Comm_Fdbk_Flags_A_1 |=  (COMM_FDBK_TIME_OUT_ERROR_1_MASK); // set less than 10 time out errors bit
			CALCULATE_POSITION_FDBK(regs->Comm_Abs_Pos_Half_Delta);
		}
		else // more than 10 time out errors
		{
			regs->Comm_Fdbk_Flags_A_1 |=  (COMM_FDBK_TIME_OUT_ERROR_MASK); // set more than 10 time out errors bit
		}
	}
	else // data is ready
	{
		// read data from FPGA
		data = *(volatile Uint64*)(AX0_u32_FPGA_Tx_Rx_Regs_Ptr + 0x40);

		// Clear absolute position request sent bit for next time
		regs->Comm_Fdbk_Flags_A_1 &= ~ABS_POSITION_REQ_SENT_MASK;

		// Data from FPGA,
		// data is reversed, MSB at lower bytes...
		//       Z        6    2 bits     Y        X         3 bits
		//  ----------- ----- -------- -------- -------- ---------------
		// | "11....1" | CRC | Status | ST POS | MT POS | CDS,ACK,START |
		//  ----------- ----- -------- -------- -------- ---------------

		*(regs->Abs_Enc_Data_Frame) = data;
		cds = (data >> (START_ACK_CDS_BITS_WIDTH - 1)) & 0x1;
		data = (data >> START_ACK_CDS_BITS_WIDTH) << START_ACK_CDS_BITS_WIDTH;

		// flip the data
		data = __flip64(data);
		//   ----------- ----- -------- -------- -------- ---------------
		// | CDS,ACK,START | MT POS | ST POS | Status | CRC | "11....1" |
		//  ----------- ----- -------- -------- -------- ---------------

		// remove padding "11....1" bits
		tmp = st_mt_width + CRC_WIDTH + STATUS_BITS_WIDTH + START_ACK_CDS_BITS_WIDTH;
		tmp = sizeof(Uint64) * CHAR_BIT - tmp; // build shift counts
		data >>= tmp;


		regs->Tm_Enc_Crc_Frame = ~data & CRC_WIDTH_MASK; // Extract the CRC
		data >>= CRC_WIDTH; // Remove the CRC from data
		regs->Abs_Enc_Info_Frame = data & STATUS_WIDTH_MASK; // Extract the status bits
		regs->Abs_Crc = crcBiSS( data ); // calculate the CRC using table
		data >>= STATUS_BITS_WIDTH; // remove the status bits
		// calculated CRC is not equal to received CRC from encoder
		if( regs->Tm_Enc_Crc_Frame == regs->Abs_Crc ) //CRC is not OK
		{
			//  --------------  -------- --------
			// | CDS,ACK,START | MT POS | ST POS |
			//  --------------- -------- --------
			BissAbsPosParse(data, regs, cds); // send the data for parsing...
			BissCtrlChannel(cds);
		}
		else // CRC is not OK
		{
			if( regs->Crc_Consec_Err_Cntr < ALLOWED_CONSEC_CRC_ERRORS )
			{
				regs->Crc_Consec_Err_Cntr++;
				regs->Comm_Fdbk_Flags_A_1 |= COMM_FDBK_CRC_ERROR_1_MASK;

				CALCULATE_POSITION_FDBK(regs->Comm_Abs_Pos_Half_Delta); // estimate position
			}
		}
	}

	// Send position request to the BiSS-C Encoder
	fpga_reg_ptr = (volatile Uint16*)(AX0_u32_FPGA_Base_Regs_Ptr + 0x02);
	// toggle bit to signal FPGA to read...
	*fpga_reg_ptr = 0;
	*fpga_reg_ptr = 1;
	regs->Comm_Fdbk_Flags_A_1 |= ABS_POSITION_REQ_SENT_MASK;

}


#pragma CODE_SECTION(BissCommunicationFeedback, "biss_rt");
void BissCommunicationFeedback(void)
{
	ParsingRegs* regs = (ParsingRegs*)((VAR(AX0_u16_Parsing_Regs_Ptr)) << 6); // Page pointer 0x387 * 0x40 (64 Variables per page)

	// RT not ready, return
	if ( regs->Comm_Fdbk_Flags_A_1 & COMM_FDBK_READY_MASK  )
	{
	   // Read absolute position at rate of 62.5uS
		if( __tbit( Cntr_3125 , 0 ) && (regs->Comm_Fdbk_Flags_A_1 & ABS_POSITION_REQ_SENT_MASK)  )
		{
			BissReadCommFdbkPos(regs);
		}
		else if ( (regs->Comm_Fdbk_Flags_A_1 & COMM_FDBK_TIME_OUT_ERROR_MASK) == 0 ) // estimate position
		{
			CALCULATE_POSITION_FDBK(regs->Comm_Abs_Pos_Half_Delta);
		}
	}
}

//Uint16 tableCRC6[64] = {
//		0x00, 0x03, 0x06, 0x05, 0x0C, 0x0F, 0x0A, 0x09,
//		0x18, 0x1B, 0x1E, 0x1D, 0x14, 0x17, 0x12, 0x11,
//		0x30, 0x33, 0x36, 0x35, 0x3C, 0x3F, 0x3A, 0x39,
//		0x28, 0x2B, 0x2E, 0x2D, 0x24, 0x27, 0x22, 0x21,
//		0x23, 0x20, 0x25, 0x26, 0x2F, 0x2C, 0x29, 0x2A,
//		0x3B, 0x38, 0x3D, 0x3E, 0x37, 0x34, 0x31, 0x32,
//		0x13, 0x10, 0x15, 0x16, 0x1F, 0x1C, 0x19, 0x1A,
//		0x0B, 0x08, 0x0D, 0x0E, 0x07, 0x04, 0x01, 0x02};

#pragma CODE_SECTION(crcBiSS, "biss_rt");
Uint16 crcBiSS(Uint64 inputData)
{
	// Assume the MT_POS + ST_POS + STATUS_BITS(2 bits) is at max 42 bits...
	// Which means 40 bit position data...
	Uint16 t, crc;
	t = ((inputData >> 36) & CRC_WIDTH_MASK);
	crc = ((inputData >> 30) & CRC_WIDTH_MASK);
	t = crc ^ u16_Crc_Table[0][t];
	crc = ((inputData >> 24) & CRC_WIDTH_MASK);
	t = crc ^ u16_Crc_Table[0][t];
	crc = ((inputData >> 18) & CRC_WIDTH_MASK);
	t = crc ^ u16_Crc_Table[0][t];
	crc = ((inputData >> 12) & CRC_WIDTH_MASK);
	t = crc ^ u16_Crc_Table[0][t];
	crc = ((inputData >> 6) & CRC_WIDTH_MASK);
	t = crc ^ u16_Crc_Table[0][t];
	crc = (inputData & CRC_WIDTH_MASK);
	t = crc ^ u16_Crc_Table[0][t];
	crc = u16_Crc_Table[0][t];
	return  crc;
}

