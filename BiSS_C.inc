;/*
; * BiSS_C.inc
; *
; *  Created on: 14.04.2015
; *      Author: anatoly.odler
; */

; -------------------------------------------
; BiSS-C RT mailbox bits
; -------------------------------------------
BISSC_RTMBX_REQ_READY_BIT                 .set    0
BISSC_RTMBX_RSP_READY_BIT                 .set    1
BISSC_RTMBX_CNTRL_CH_TX_STATE_BIT         .set    2
BISSC_RTMBX_CNTRL_CH_RX_STATE_BIT         .set    3
BISSC_RTMBX_CNTRL_CH_START_BIT_EVALUATING .set    4
; More bits here to manage the TX/RX state machines
; -------------------------------------------

; -------------------------------------------
; BiSS-C Control Channel Management Constants
; -------------------------------------------
BISSC_CNTRL_CH_REQUEST_BITS_TO_FETCH            .set     47
BISSC_CNTRL_CH_REQUEST_BITS_TO_SKIP             .set     14 ; [CDMs]
BISSC_CNTRL_CH_REQUEST_BITS_TO_EVAL_START_BIT   .set     34 ; [CDMs]

; -------------------------------------------
; BiSS-C profile definitions
; -------------------------------------------
BISSC_PRFL_RSRVD0                   .set    0
BISSC_PRFL_BP1                      .set    1
BISSC_PRFL_RSRVD2                   .set    2
BISSC_PRFL_BP3                      .set    3

BISSC_PRFL_BP1_24_24                .set    0
BISSC_PRFL_BP1_12_24                .set    1
BISSC_PRFL_BP1_24_12                .set    2
BISSC_PRFL_BP1_0_24                 .set    3
BISSC_PRFL_BP1_12_12                .set    4
BISSC_PRFL_BP1_24_0                 .set    5
BISSC_PRFL_BP1_0_12                 .set    6
BISSC_PRFL_BP1_12_0                 .set    7


; -------------------------------------------
; BiSS-C status definitions
; -------------------------------------------
BISSC_STATUS_FAULT_BIT              .set    0
BISSC_STATUS_WARN_BIT               .set    1

