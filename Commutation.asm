;/*
; * Commutation.asm
; *
; *  Created on: Oct 8, 2019
; *      Author: vadim.malinovsky
; */
 .include "currloop_0.var"
 .include "currloop.inc"
 .include "BiSS_C.inc"

LDP        .macro P1
      	    MOVW DP,#P1
      		.endm

; Set DSP flags that might be modified by C code
C_RETURN_RESTORE .macro
   SETC     SXM
   CLRC     OVM
   SPM      #0
   .endm


 .global _CALCULATE_POSITION_FDBK
 .global _BISSC_COMMUNICATION_FEEDBACK
 .global _crcBiSS
 .global _foo_asm


 .global  _Cntr_3125
 .global  _AX0_u32_Cumul_TO_Err_Cntr
 .global  _AX0_u32_Cumul_Crc_Err_Cntr
 .global  _u16_Crc_Table

 .asg "*-SP[4]",DATA0
 .asg "*-SP[3]",DATA1
 .asg "*-SP[2]",DATA2
 .asg "*-SP[1]",DATA3

_foo_asm: .asmfunc
   MOVW     DP,#AX0_CRRNT_5_VAR_PAGE
   PUSH     @_AX0_u16_Parsing_Regs_Ptr
   MOV      AL,*-SP[1] ; save the dp pointer
   LSL      AL,#6
   MOV      AH,AL
   POP      DP

   MOVL     XAR4,#_AX0_u32_FPGA_Tx_Rx_Regs_Ptr ; Load XAR2 with Address of _AX0_u32_FPGA_Tx_Rx_Regs_Ptr
   MOVL     XAR5,*XAR4                         ; HDSL_POS4_ADDR

   MOV      AR4,#_AX0_u16_Abs_Enc_Data_0Frame
   AND      AL,AR4,#0x3f
   OR       AL,AH
   MOV      AR4,AL

   ; first read
   MOV      PL,*+XAR5[0] ; read from 0x4810, data from 0x4811:0x4810
   MOV      PL,*+XAR5[0] ; we will get AL.MSB:AL.LSB = POS3:POS4

   ; second read
   MOV      PH,*+XAR5[2] ; read from 0x4812, data from 0x4813:0x4812
   MOV      PH,*+XAR5[2] ; we will get AL.MSB:AL.LSB = POS1:POS2


   ; third read
   MOV      AL,*+XAR5[4] ; read from 0x4814, data from 0x4815:0x4814
   MOV      AL,*+XAR5[4] ; we will get AL.MSB:AL.LSB = VEL2:POS0

   MOVB     *+XAR4[0],AL.LSB
   MOV      AL,PH
   MOVB     *+XAR4[1],AL.MSB
   MOVB     *+XAR4[2],AL.LSB
   MOV      AL,PL
   MOVB     *+XAR4[3],AL.MSB
   MOVB     *+XAR4[4],AL.LSB

	LRETR
	.endasmfunc

_BISSC_COMMUNICATION_FEEDBACK: .asmfunc stack_usage(4)
   ADDB     SP,#4
   MOVW     DP,#AX0_CRRNT_5_VAR_PAGE
   PUSH     @_AX0_u16_Parsing_Regs_Ptr
   POP      DP
   TBIT     @_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_READY_BIT
   BF       SKIP_BISSC_POSITION_REQ_SEND,NTC

   MOVL     XAR4,#_Cntr_3125
   TBIT     *XAR4,#0                  ; Read abs pos at rate of 62.5uS
   SB       READ_BISSC_COMM_FDBK_POS,TC

BISSC_COMMUNICATION_FEEDBACK_ESTIMATE_POS:
   TBIT     @_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_TIME_OUT_ERROR_SET
   BF       BISSC_COMMUNICATION_FEEDBACK_END,TC

   MOVL     ACC,@AX0_s32_Comm_Abs_Pos_Half_Delta ; Position Estimation (1/2) Delta
   LCR      _CALCULATE_POSITION_FDBK
   BF       BISSC_COMMUNICATION_FEEDBACK_END,UNC

READ_BISSC_COMM_FDBK_POS:
   TCLR     @_AX0_u16_Comm_Fdbk_Flags_A_1,#ABS_POSITION_REQ_SENT_BIT ; Check if request sent
   BF       BISSC_COMMUNICATION_FEEDBACK_END,NTC

   MOVL     XAR2,#_AX0_u32_FPGA_Base_Regs_Ptr
   MOVL     XAR1,*XAR2
   ADDB     XAR1,#0000Bh
   TBIT     *XAR1,#1    ; Check if FPGA received data
   SB       READ_BISSC_POS_DATA_FROM_FPGA,TC

   CMP      @_AX0_u16_Time_Out_Consec_Err_Cntr,#10 ; Detect ten consecutive Time-Out Errors
   BF       BISSC_TIME_OUT_ERROR_SET,GT
   INC      @_AX0_u16_Time_Out_Consec_Err_Cntr ; Increment Time-Out Consecutive Event Counter

   MOVL     XAR4,#_AX0_u32_Cumul_TO_Err_Cntr ; Debug only!!!!!!!!!!!!!!!!!!!!!
   MOVB     ACC,#1                           ; Debug only!!!!!!!!!!!!!!!!!!!!!
   ADDL     *XAR4,ACC  						 ; Debug only!!!!!!!!!!!!!!!!!!!!!

   TSET     @_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_TIME_OUT_ERROR_1_BIT
   MOVL     ACC,@AX0_s32_Comm_Abs_Pos_Half_Delta
   LCR      _CALCULATE_POSITION_FDBK
   BF       BISSC_COMMUNICATION_FEEDBACK_END,UNC
BISSC_TIME_OUT_ERROR_SET:
   TSET     @_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_TIME_OUT_ERROR_SET
   BF       BISSC_COMMUNICATION_FEEDBACK_END,UNC

;****************************************************************************
; Here we collect the data from the FPGA registers.
; We load the data words with FPGA RX buffer.
;****************************************************************************
READ_BISSC_POS_DATA_FROM_FPGA:
   MOVL     XAR7, #_AX0_u32_BiSSC_MT_ST_Length ; T holds the length of MT and ST
   MOV      T,*XAR7
   MOV      @_AX0_u16_Abs_Crc,#0     ; Init CRC

   ; First we collect the received data into the data words
   MOVL     XAR4,#_AX0_u32_FPGA_Tx_Rx_Regs_Ptr
   MOVL     XAR5,*XAR4
   ADDB     XAR5,#40h; Receive Registers based on Transmit Registers' Address

   MOVL     P,*XAR5++
   MOV      @_AX0_u16_Abs_Enc_Data_0Frame,P; ACK, Start, CDS and MSBits of Position Data
   MOVH     @_AX0_u16_Abs_Enc_Data_1Frame,P; Rest of Position Data
   MOVL     ACC,*XAR5
   MOV      @_AX0_u16_Abs_Enc_Data_2Frame,ACC ; Additional Position data, Status Bits and CRC
   MOVH     @_AX0_u16_Abs_Enc_Data_3Frame,ACC

;****************************************************************************
; Here we reflect the received data (TODO: Let fpga do the reflect part...)
;****************************************************************************
   FLIP     AL
   FLIP     AH
   MOV      PH,AL
   MOV      PL,AH
   MOV      AL,@_AX0_u16_Abs_Enc_Data_1Frame
   MOV		AH,@_AX0_u16_Abs_Enc_Data_0Frame
   FLIP     AL
   FLIP     AH
;****************************************************************************
; End of Reflect
;****************************************************************************

   LSL64    ACC:P,#3  ; Remove Ack, Start, and CDS Bits
   MOVL     DATA0,P    ;  data frames on DATA3:DATA2:DATA1:DATA0
   MOVL     DATA2,ACC
   MOVB     ACC,#64
   MOV      AH,T   ; AH = T= mt_st_len bits
   ADD      AH,#8 ; += Warning + Error + 6 Crc Bits
   SUB      AL,AH
   MOV      T,AL
   ;MOVL     P,XAR5
   MOVL     ACC,DATA2
   LSR64    ACC:P,T    ; Remove padding "1111...11" bits
;****************************************************************************
; Extract the Received CRC (MSB first)
;****************************************************************************
   MOV      AR0,AL	   ; save AL
   MOV      AL,@PL
   NOT      AL
   AND      AL,#0003Fh                  ; Isolate CRC Bits
   MOV      @AX0_u16_Tm_Enc_Crc_Frame,AL   ; Store received CRC value
   MOV      AL,AR0       ; restore AL
   LSR64    ACC:P,#6     ; Remove CRC bits
   ; save position data to XAR2:XAR1
   MOVL     DATA0,P
   MOVL     DATA2,ACC
;**************************************************************************************
; Calculate the CRC from the data:  all data except ACK, Start, CDS, CRC and Stop bits.
;**************************************************************************************
   LCR      _crcBiSS ; Uint16 crcBiSS(Uint64 inputData)
   C_RETURN_RESTORE
	; returned calculated crc is in AL
;****************************************************************************
; Compare the recevied (inverted) and the calcualted CRC values
;****************************************************************************
BISSC_CRC_CHECK:
   MOV      @_AX0_u16_Abs_Crc,AL
   CMP      AL, @AX0_u16_Tm_Enc_Crc_Frame
   SB       BISSC_CRC_OK,EQ

   CMP      @_AX0_u16_Crc_Consec_Err_Cntr,#10 ; Detect ten consecutive CRC Errors
   BF       BISSC_FIRST_CRC_ERR,LT
   TSET     @_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_CRC_ERROR_SET
   BF       BISSC_COMMUNICATION_FEEDBACK_END,UNC

BISSC_FIRST_CRC_ERR:
   INC      @_AX0_u16_Crc_Consec_Err_Cntr    ; Increment Counter of Consecutive CRC Errors
   TSET     @_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_CRC_ERROR_1_BIT
   MOVL     XAR4,#_AX0_u32_Cumul_Crc_Err_Cntr
   MOVB     ACC,#1
   ADDL     *XAR4,ACC
   MOVL     ACC,@AX0_s32_Comm_Abs_Pos_Half_Delta ; estimate position
   LCR      _CALCULATE_POSITION_FDBK
   BF       BISSC_COMMUNICATION_FEEDBACK_END,UNC

BISSC_CRC_OK:
; Extract the Status bits
   MOV      AL,DATA0 ; DATA0 holds | mt pos partially |st pos|status bits|
   NOT		AL
   AND		AL,#0x3
   MOV		@_AX0_u16_Abs_Enc_Info_Frame,AL
   ; Pos data in ACC:P
   MOVL		P,DATA0
   MOVL     ACC,DATA2
   LSR64	ACC:P,#2	; remove status bits
   MOV      AR0,AL	    ; save AL
   MOVL     XAR4,#_AX0_u32_BiSSC_MT_ST_Length_Resolution ; contains | mt_total | mt_eff | st_total | st_eff |
   MOVB     AL.LSB,*+XAR4[1]
   MOV      AR5,AL ; save st_total for later
   MOV      T,#32
   SUB      T,AL
   MOV      AL,AR0		; restore AL
   LSL64    ACC:P,T ; ACC holds mt_total data Right-Justified
   AND      AL,@_AX0_u32_Biss_Multi_Turn_Eff_Mask
   AND      AH,@_AX0_u32_Biss_Multi_Turn_Eff_Mask+1 ; mask out only the effiecient part
   MOVL     DATA2,ACC     ; Save mt_total data
   MOVL     ACC,P
   LSRL     ACC,T ; ACC holds st_total Data Right-Justified
   AND      AL,@_AX0_u32_Biss_Single_Turn_Eff_Mask ; mask out only the effiecient part
   AND      AH,@_AX0_u32_Biss_Single_Turn_Eff_Mask
   MOVL     DATA0,ACC     ; Save st_total data

BISSC_HANDLE_32_BIT_RES:
   MOV      AL,AR5 ; AL holds st_eff
   BF       BISSC_LESS_32_BIT_RES_ABS_POS,NEQ ; if ( (32 - SingleTurnRes) == 0 )
   MOV      AL,#2                     ; else // (SignleTurnRes == 32), Shr >> 2, for 30-bit res
BISSC_LESS_32_BIT_RES_ABS_POS:
   MOV      AL,#0
   MOV      T,@AL
   MOVL     ACC,DATA0
   MOVL     XAR4,#_AX0_u32_BiSSC_Abs_Pos_Raw ; Pos Raw holds data without reduction
   MOVL     *XAR4, ACC
   LSRL     ACC,T ; ACC holds ST Data Right-Justified

   TCLR		@_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_INIT_ABS_POS_BIT
   BF		BISSC_ST_POS_PARSE, NTC

;****************************************************************************
; This section contains initialization of the absolute feedback position
;****************************************************************************
BISSC_ABS_POS_PARSE_INIT:
   MOVL     P,DATA2
   MOVL     @_AX0_s32_Comm_Abs_Pos_32_Hi,P		; initialize MT data bits
   MOVL     @_AX0_s32_Pos_Fdbk_Hi,P
   MOVL     @_AX0_u32_Comm_Abs_Pos_32_Lo,ACC
   MOVL     @_AX0_u32_Fdbk_Accu,ACC

   MOVB     ACC,#0
   MOVL     @AX0_s32_Comm_Abs_Pos_Half_Delta,ACC
   LCR      _CALCULATE_POSITION_FDBK
   BF       BISSC_COMMUNICATION_FEEDBACK_END,UNC

;****************************************************************************************
; This section contains the cyclic parsing of the absolute feedback in-turn position data
;****************************************************************************************
BISSC_ST_POS_PARSE:
READ_BISSC_CNTRL_CH_DATA:
   LDP      AX0_CRRNT_6_VAR_PAGE
   TBIT     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_RX_STATE_BIT
   SB       BISSC_NO_CDS_RESPONSE, NTC

   ; Check if we are on the start bit evaluation state
   TBIT     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_START_BIT_EVALUATING
   SB       BISSC_NO_CDS_RESPONSE, TC

BISSC_COLLECT_CDS_RESPONSE:
; Extract the CDS bit from the received data
; First shift the response frame for 1 to the left, making room to the next CDS to come
   MOVL     ACC,@_AX0_u32_BiSSC_ResponseCmdFrame
   MOV      T,#1
   LSLL     ACC,T
   MOVL     @_AX0_u32_BiSSC_ResponseCmdFrame,ACC
; If the CDS bit is high, then we set the LSB of the response frame
; Do nothing otherwise
   MOVL     XAR4,#_AX0_u16_Abs_Enc_Data_0Frame
   TBIT     *XAR4, #2
   SB       BISSC_NOT_SET_CDS, NTC
   TSET     @_AX0_u32_BiSSC_ResponseCmdFrame, #0
BISSC_NOT_SET_CDS:
; Check if we are finished processing control channel request
   CMP      @_AX0_u16_BiSSC_CDMCntr, #BISSC_CNTRL_CH_REQUEST_BITS_TO_FETCH
   SB       BISSC_NO_CDS_RESPONSE, LT

   AND      @_AX0_u16_BiSSC_RTMBX_Control, #~(1 <<BISSC_RTMBX_CNTRL_CH_TX_STATE_BIT | 1 << BISSC_RTMBX_CNTRL_CH_RX_STATE_BIT)
   ;TCLR     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_RX_STATE_BIT
   TSET     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_RSP_READY_BIT

BISSC_NO_CDS_RESPONSE:
UPDATE_BISSC_PFB_COMM_FDBK:
   MOVW     DP,#AX0_CRRNT_5_VAR_PAGE
   PUSH     @_AX0_u16_Parsing_Regs_Ptr
   POP      DP

   MOVL     P,ACC
   SUBUL    ACC,@_AX0_u32_Comm_Abs_Pos_32_Lo   ; acc = pos - prev_pos

   ; Handle Rollver
   MOVL     XAR2,@ACC
   ABS      ACC
   CMPL     ACC,@_AX0_u32_Counts_Per_Rev_Half
   BF       NO_BISSC_ROLLOVER,LT
   MOVL     ACC,@XAR2
   BF       BISSC_POS_ROLLOVER,GT
BISSC_NEG_ROLLOVER:
   ADDL     ACC,@_AX0_u32_Counts_Per_Rev
   BF       AFTER_BISSC_ROLLOVER_HANDLING,UNC
BISSC_POS_ROLLOVER:
   SUBL     ACC,@_AX0_u32_Counts_Per_Rev
   BF       AFTER_BISSC_ROLLOVER_HANDLING,UNC
NO_BISSC_ROLLOVER:
   MOVL     ACC,@XAR2
AFTER_BISSC_ROLLOVER_HANDLING:
   MOVL     @_AX0_u32_Comm_Abs_Pos_32_Lo,P
   NEG      ACC
   MOVL     P,@AX0_s32_Comm_Abs_Pos_Half_Delta ; P = Previous position estimation delta
   MOVL     XAR1,@ACC                      ; Store Difference from previous Pos. Read.

   MOV      AL,#COMM_FDBK_ALL_ERROR_MASK
   AND      AL,@_AX0_u16_Comm_Fdbk_Flags_A_1
   BF       NO_NEW_HALF_DELTA_BISSC,NEQ   ; Avoid calculating new Half_Delta if
   MOVL     ACC,XAR1                      ; recovering from any Interpolation Condition
   SFR      ACC,#1                  ; Calculate new position estimation delta, 1/2 because
                                    ; sampling at 16,000 times per second, 62.5usec
;   SFR      ACC,#2                  ; Calculate new position estimation delta, 1/4 because
                                    ; sampling at 8,000 times per second, 125 usec
   MOVL     @AX0_s32_Comm_Abs_Pos_Half_Delta,ACC
NO_NEW_HALF_DELTA_BISSC:
   MOVL     ACC,XAR1
   SUBL     ACC,@P                ; Subtract the 1 time 1/2 Delta was applied, 62.5usec

   ;TCLR     @_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_POS_INTERPOLATED
   CMP      @_AX0_u16_Crc_Consec_Err_Cntr,#0 ; Check the number of previous Consecutive CRC Errors
   BF       BISSC_CALC_CRC_TO_ERR_COMP,NEQ      ; Calculations if CRC Errors
   CMP      @_AX0_u16_Time_Out_Consec_Err_Cntr,#0 ; Check number of previous Time-Out Errors
   BF       BISSC_SKIP_CRC_TO_ERR_COMP,EQ
BISSC_CALC_CRC_TO_ERR_COMP: ; Calculate CRC and Time Out Error Compensation
   MOVL     XAR1,@ACC                       ; Temporary storage of Motion Delta
   MOVL     XAR2,@P                         ; Temporary storage of Half_Delta
   MOV      ACC,@_AX0_u16_Crc_Consec_Err_Cntr
   ADD      ACC,@_AX0_u16_Time_Out_Consec_Err_Cntr
   LSL      ACC,#1                          ; Double the Counter to sutract two Half_Deltas for
                                            ; each CRC Error 62.5 usec
   ;LSL      ACC,#2                          ; Quadruple the Counter to sutract four
                                            ; Half_Deltas for each CRC and TO Error 125usec
   MOVL     XT,@ACC                         ; XT holds Error Cntrs extended to 32 Bits
   IMPYL    P,XT,@XAR2
   MOVL     ACC,@XAR1                       ; Restore Motion Delta
   SUBL     ACC,@P                          ; Subtract correction for previous CRC Errors
   MOV      @_AX0_u16_Crc_Consec_Err_Cntr,#0 ; Zero CRC Consecutive Error Counter
   AND      @_AX0_u16_Comm_Fdbk_Flags_A_1,#~(1 << COMM_FDBK_CRC_ERROR_1_BIT | 1 << COMM_FDBK_TIME_OUT_ERROR_1_BIT)
   MOV      @_AX0_u16_Time_Out_Consec_Err_Cntr,#0 ; Zero Time-Out Consecutive Error Counter
   ;TCLR     @_AX0_u16_Comm_Fdbk_Flags_A_1,#COMM_FDBK_TIME_OUT_ERROR_1_BIT

BISSC_SKIP_CRC_TO_ERR_COMP:
   MOV      @AX0_s16_Gen_Delta,AL            ; Store delta to variable
   LCR      _CALCULATE_POSITION_FDBK

;****************************************************************************
; Decide if a request to read data has to be sent in this cycle
;****************************************************************************
BISSC_COMMUNICATION_FEEDBACK_END:
   MOVL     XAR4,#_Cntr_3125
   TBIT     *XAR4,#0                  ; Read abs pos at rate of 62.5 uS
   SB       SKIP_BISSC_POSITION_REQ_SEND,NTC

; Here we need to define a state machine, which will set appropripate CDM bit according to the _AX0_u32_BiSSC_RegCmdFrame,
; which is loaded in BG Manager.
; Consider to use the same state machine for response or make another state machine.
; Response state machine may appear right after all FPGA registers are read in READ_BISSC_POS_DATA_FROM_FPGA
; The response state machine will set the response bits in _AX0_u32_BiSSC_ResponseCmdFrame. At the end of transmission,
; it will indicate to BG that response is ready by setting BISSC_RTMBX_RSP_READY_BIT in _AX0_u16_BiSSC_RTMBX_Control.
BISSC_SENDING:
   LDP      AX0_CRRNT_6_VAR_PAGE
   TBIT     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_TX_STATE_BIT
   SB       BISSC_CDM_SENDING_STATE, TC

   TCLR     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_REQ_READY_BIT
   SB       BISSC_SEND_REQUEST, NTC
   TSET     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_TX_STATE_BIT
   MOV      @_AX0_u16_BiSSC_CDMCntr, #0

BISSC_CDM_SENDING_STATE:
   MOVL     XAR2,#_AX0_u32_FPGA_Base_Regs_Ptr
   MOVL     XAR1,*XAR2
   ADDB     XAR1,#0003Ch
   CMP      @_AX0_u16_BiSSC_CDMCntr, #BISSC_CNTRL_CH_REQUEST_BITS_TO_SKIP; Transmit 14 zeroes on CDM first
   SB       BISSC_CNTR_INCREMENT, LT

; Check if we are on the start bit evaluation state
   TBIT     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_START_BIT_EVALUATING
   SB       BISSC_EVAL_START_BIT_TX, TC

; Check if evaluation of the start bit can be executed
   CMP      @_AX0_u16_BiSSC_CDMCntr, #BISSC_CNTRL_CH_REQUEST_BITS_TO_EVAL_START_BIT
   SB       BISSC_SET_CDM_DATA, NEQ

; Set start bit evaluation state
BISSC_SET_EVAL_START_BIT:
   TSET     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_START_BIT_EVALUATING
   MOVL     ACC, @_AX0_u32_BiSSC_MT_ST_Length_Resolution ; CDM start bit control workaround
   LSR      AH, #8                                       ; CDM start bit control workaround
   SBF      BISSC_START_FPGA_START_BIT_EVALUATION, EQ    ; CDM start bit control workaround
BISSC_SET_CDM_DIRECT:                                    ; CDM start bit control workaround
   MOV      *XAR1, #1h                                   ; CDM start bit control workaround
   SB       BISSC_EVAL_START_BIT_TX, UNC                 ; CDM start bit control workaround
BISSC_START_FPGA_START_BIT_EVALUATION:                   ; CDM start bit control workaround
   TSET     *XAR1, #1   ; CDM Start bit recognition

BISSC_EVAL_START_BIT_TX:
   MOVW     DP,#AX0_CRRNT_5_VAR_PAGE
   PUSH     @_AX0_u16_Parsing_Regs_Ptr
   POP      DP
   TBIT     @_AX0_u16_Abs_Enc_Data_0Frame, #2
   SB       BISSC_SEND_REQUEST, NTC

BISSC_START_BIT_FOUND_TX:
   LDP      AX0_CRRNT_6_VAR_PAGE
   MOVL     XAR2,#_AX0_u32_FPGA_Base_Regs_Ptr
   MOVL     XAR1,*XAR2
   ADDB     XAR1,#0003Ch
   MOVL     ACC, @_AX0_u32_BiSSC_MT_ST_Length_Resolution ; CDM start bit control workarround
   LSR      AH, #8                                       ; CDM start bit control workarround
   SBF      BISSC_NO_STOP_FPGA_START_BIT_EVALUATION, EQ  ; CDM start bit control workarround
BISSC_STOP_FPGA_START_BIT_EVALUATION:                    ; CDM start bit control workarround
   TCLR     *XAR1, #1
BISSC_NO_STOP_FPGA_START_BIT_EVALUATION:                    ; Anatoly CDM start bit control workarround
   TCLR     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_START_BIT_EVALUATING

BISSC_SET_CDM_DATA: ; Transmits the actual payload on the CDM
; Move request data to CDM
   TBIT     @_AX0_u32_BiSSC_RegCmdFrame, #0
   MOVB     *XAR1, #0h, NTC
   MOVB     *XAR1, #1h, TC

BISSC_SET_CDM_DATA_CONT: ; Transmits the actual payload on the CDM
; Shift request data for next cycle
   MOV      T,#1
   MOVL     ACC, @_AX0_u32_BiSSC_RegCmdFrame
   LSRL     ACC, T
   MOVL     @_AX0_u32_BiSSC_RegCmdFrame,ACC
   ; Increment the CDM counter
   INC      @_AX0_u16_BiSSC_CDMCntr
   TSET     @_AX0_u16_BiSSC_RTMBX_Control, #BISSC_RTMBX_CNTRL_CH_RX_STATE_BIT
   SB       BISSC_SEND_REQUEST,UNC

BISSC_CNTR_INCREMENT:   ; Transmits zero CDM bit to reset control channel communcation
   MOV      *XAR1, #0h
   INC      @_AX0_u16_BiSSC_CDMCntr

;Send position request to the BiSS-C Encoder
BISSC_SEND_REQUEST:
   MOVL     XAR2,#_AX0_u32_FPGA_Base_Regs_Ptr
   MOVL     XAR1,*XAR2
   ADDB     XAR1,#00002h
   MOV      *XAR1,#0h
   MOV      *XAR1,#1h

   MOVL	    XAR4,#_AX0_u16_Comm_Fdbk_Flags_A_1
   TSET     *XAR4,#ABS_POSITION_REQ_SENT_BIT

SKIP_BISSC_POSITION_REQ_SEND:
   SUBB SP,#4
   LRETR
   .endasmfunc

_CALCULATE_POSITION_FDBK: .asmfunc
	ADD ACC,#1
	LRETR
	.endasmfunc





BISSC_FLIP_FRAMES: .asmfunc; Uint64 BISSC_FLIP_FRAMES(Uint64 data)
; Before flip:
; | AH[63:48] | AL[47:32] | PH[31:16] | PL[15:0] |
;
; After  flip each 16 bit register:
; | AH[48:63] | AL[32:47] | PH[16:31] | PL[0:15] |
;
; Need rearrange, swap:
;  AH <-> PL && AL <-> PH

   PUSH     P
   ; flip
   FLIP     AH
   FLIP     AL
   ; Rearrange
   MOV      PH,AL
   MOV      PL,AH
   ; Restore PL -> AL, PL -> AL
   POP      ACC
   ; Rearange, Swap AL,AH, PL -> AH, PH -> AL
   MOV      AR0,AL
   MOV      AL,AH
   MOV      AH,AR0
   ; flip
   FLIP     AL
   FLIP     AH
   LRETR
   .endasmfunc
