; General Definitions
; -------------------
SIN_TBL_RES                     .set   7h

; Product Definitions
; -------------------
SHNDR                           .set 1
DDHD                            .set 2
GBOT10                          .set 3
SHNDR_HW                        .set 4
EC_LITE                         .set 5
BDHD                            .set 6


; skip flags definitions
; ----------------------
SKIP_ENC_STATE_2_BIT            .set    0
HALLS_ONLY_BIT                  .set    1
ENC_CONFIG_NEEDED_BIT           .set    2
SKIP_COMM_ADJ_BIT               .set    3
SPD_ADV_INV_DIRECTION_BIT       .set    4
ZERO_PHASE_ADV_BIT              .set    5
MOTOR_SETUP_BIT                 .set    6
SWAP_UV_COMM_BIT                .set    7 ; Phases and feedback inversion
INV_DIRECTION_BIT               .set    8
WNS_ON_BIT                      .set    9
SKIP_MENCZPOS_USE_BIT           .set    10
INIT_RESOLVER_PFB_BIT           .set    11
UPDATE_RESOLVER_PFB_BIT         .set    12
SW_SINE_UPDATING_PFB_BIT        .set    13
SL_COMMUTATION_BIT              .set    14
PS_ZERO_PFB_BIT                 .set    15
; AX0_s16_Home_Skip_Flags definitions
; --------------------------
SKIP_PFB_COPY_TO_PCMD_BIT  			.set    0

; Crrnt_Run_Code definitions
; --------------------------
MOTOR_SETUP_LOOK_FOR_INDEX_BIT  .set    0
MOTOR_SETUP_INDEX_CAPTURED_BIT  .set    1
HOMING_SOFT_INDEX_BIT           .set    2
RESET_SWR2D_BIT                 .set    3
ARM_HOMING_CAPTURE_BIT          .set    4
COPY_POS_CMD_TO_PTP_GEN_BIT     .set    5
RES_EXC_INITIALIZED_BIT         .set    6
COPY_SHARED_POS_COEF_BIT        .set    7
HOMING_OFFSET_READY_BIT         .set    8
COPY_POS_COEF_BIT               .set    9
COPY_NCT_COEF_BIT               .set    10
COPY_VEL_COEF_BIT               .set    11
SOFT_HOMING_CAPTURE_BIT         .set    12
PHASE_FIND_ORG_BIT              .set    13
LOOK_FOR_INDEX_BIT              .set    14
INDEX_CAPTURE_DONE_BIT          .set    15


; Sine Enc bits
; --------------
SIN_NEG_DONE_BIT                .set    0
SIN_POS_DONE_BIT                .set    1
COS_NEG_DONE_BIT                .set    2
COS_POS_DONE_BIT                .set    3
REVERSE_SIN_COS_BIT             .set    4
SINE_CD_INIT_DONE_BIT           .set    5
SINE_ZERO_VEL_TOO_HIGH_BIT      .set    6
INV_CRRNT_CMD_BIT               .set    7
SW_SINE_READ_POS_ENA_BIT        .set    8
SW_SINE_READ_POS_DONE_BIT       .set    9
SW_SINE_QUAD_MISMATCH_BIT       .set    10
SW_SINE_POS_ESTIMATED           .set    11
SW_SINE_INIT                    .set    12
SINE_ZERO_DONE                  .set    13
SINE_ZERO_RUNNING               .set    14
RESET_SINE_ZERO                 .set    15


; AX0_u16_Comm_Fdbk_Flags_A_1 Communication position feedback only bits
; --------------
; Bits 0,1,2 Is sync information
COMM_FDBK_INIT_ABS_POS_BIT          .set    3
COMM_FDBK_READY_BIT                 .set    4
COMM_FDBK_CRC_ERROR_1_BIT           .set    5
COMM_FDBK_TIME_OUT_ERROR_1_BIT      .set    6
ABS_POSITION_REQ_SENT_BIT           .set    7
COMM_FDBK_INIT_ABS_POS_IN_REV_BIT   .set    8
COMM_FDBK_SK_MULTI_TURN_RESET_BIT   .set    9
COMM_FDBK_SK_MULTI_TURN_RST_OFF_BIT .set    10
COMM_FDBK_POS_INTERPOLATED          .set    11
COMM_FDBK_CRC_ERROR_SET             .set    12
COMM_FDBK_ALTERN_CMD_SEND_BIT       .set    13
COMM_FDBK_ALTERN_RESP_RCV_BIT       .set    14
COMM_FDBK_TIME_OUT_ERROR_SET        .set    15

COMM_FDBK_ALL_ERROR_MASK            .set    ( (1 << COMM_FDBK_CRC_ERROR_1_BIT) | (1 << COMM_FDBK_TIME_OUT_ERROR_1_BIT) | (1 << COMM_FDBK_POS_INTERPOLATED) | (1 << COMM_FDBK_ALTERN_CMD_SEND_BIT) | (1 << COMM_FDBK_ALTERN_RESP_RCV_BIT) )

; AX0_u16_Comm_Fdbk_Flags_B Bits
; --------------
TM_CID0_PROCESS_ABSA_ONLY_BIT       .set    0
TM_AS_INCREMENTAL_ENCODER_BIT       .set    1
ANALOG_HALLS_ENCODER_BIT            .set    2
INV_DIRECTION_BIT                   .set    8


; _AX0_u16_Gear_Skip_Flags
;-------------------------
ZERO_HWPEXT_BIT                     .set    0
COPY_GEAR_FILT_BIT                  .set    4   ; Copy Gear MSQ-filter settings in real-time
COPY_HALLS_FILT_BIT                 .set    5   ; Copy Halls MSQ-filter settings in real-time
COPY_ANIN1_FILT_BIT                 .set    6   ; Copy ANIN1 MSQ-filter settings in real-time

SET_BIT_0_MASK                  .set    1
SET_BIT_1_MASK                  .set    2
FIRST_FLAG_0                    .set    14
FIRST_FLAG_1                    .set    15


; out of range bits
;------------------
AB_OUT_OF_RANGE_FLT_BIT_HI      .set    0
AB_OUT_OF_RANGE_FLT_BIT_LO      .set    1

BETA_640                        .set    045A4h
ALPHA_640                       .set    03A5Ch
ROUND_640                       .set    04000h

GLOBAL_FLAGS_MASK               .set    000Fh ; all done bits in SineZeroBit
GLOBAL_FLAGS_NUMBER             .set    15    ; all done bit form a number

;Motor encoder type
ENDAT                           .set    9
STEGMANN                        .set    10
TAMAGAWA                        .set    11


; SWR2D 1KHz Filter Parameters
; ----------------------------
; y[k] = a * y[k - 1] + b * u[k]
; a=exp(-Ts * w), b = 1 - a (unity gain)
; w[rad / sec] = 2 * pi * f = 2 * pi * 1000 , Ts = 0.0000625 sec
; => a = 0.675 , b = 0.325
; => y[k] = (22118 * y[k - 1] + 10650 * u[k]) >> 15

; 500Hz
; =>a = 0.821 , b = 0.179
; alpha = 26902 , beta = 5865

SWR2D_ALPHA                     .set    0;     6916h;  05666h
SWR2D_BETA                      .set    4000h; 16e9h;  0299Ah
SWR2D_OUT_SHR                   .set    14;     15
SWR2D_ROUND                     .set    0;      04000h


; PWM FREQ
;---------------------------------------
PWM_16KHZ_PRD                   .set    0124Fh
PWM_8KHZ_PRD                    .set    0249Eh


; SENSORLESS Filter Parameters. f = 50,100,250,500Hz
; ----------------------------
; y[k] = a * y[k - 1] + b * u[k]
; a=exp(-Ts * w) , b = 1 - a (unity gain)
; w[rad / sec] = 2 * pi * f = 2 * pi * 50 , Ts = 0.000250 sec
; w = 314.5
; => a = 0.924, b = 0.076
; => y[k] = (30277 * y[k - 1] + 2490 * u[k]) >> 15

; acc limit filter - at 100 Hz
; w[rad / sec] = 628.31
; a = 0.6965
SL_THETA_ALPHA                  .set    06D4fh;07645h;06D4fh;05666h;03A5Eh
SL_THETA_BETA                   .set    012B1h;009bah;012B0h;02999h;045A1h
SL_THETA_OUT_SHR                .set    15

; feed forward is 30 Hz single LPF at Ts = 0.000125 sec
; feed forward includes gain of G = a/(1-a) of the acc limit filter
; G(a=0.6965) = 2.29488
; w[rad/sec] = 2*pi*f = 2*pi*30 = 188.4955
; a(w=188.4955) = 0.976, b = 0.024
; y[k] = a*y[k-1] + b*u[k] = (31982*y[k-1] + 786*u[k]) >> 15
; FF = G*y[k] = y[k] * 18800 >> 13

SL_FF_ALPHA                     .set    07CEEh
SL_FF_BETA                      .set    00312h
SL_FF_OUT_SHR                   .set    15
SL_FF_ROUND                     .set    04000h

SL_FF_G_FIX                     .set    04970h
SL_FF_G_SHR                     .set    13


; Halls Only defines
;-------------------
MAX_TIME_CNTR_VAL               .set    04000h

; PLL consts
MAX_32_MTS_PLL_CORRECTION       .set    01800h
MAX_1_MTS_PLL_CORRECTION        .set    004h
PLL_LOCKED_LIMIT                .set    0100h

MUX_2V5_BIT                     .set    14
GPBDAT_LO                       .set    6FC8h
GPBDAT_HI                       .set    6FC9h
GPADAT_LO                       .set    6FC0h
GPADAT_HI                       .set    6FC1h
SCIRXDB                         .set    14

; Global gain filter 88 Hz => 5Tau = 9mSec
; Global gain filter 34 Hz => 5Tau = 23.4mSec
FGG_NLP_ALPHA                    .set    07E56h;07958h;06F7Ah
FGG_NLP_BETA                     .set    001A9h;006A8h;01086h

; LPF 200 Hz
LPF_NLP_BETA                    .set    05D7Eh
LPF_NLP_ALPHA                   .set    02282h

J_LOWER_THAN_VCMD               .set    0
USE_ACC                         .set    1
ACC_USED                        .set    2
TARGET_J_REACHED                .set    3

; Max 4Mhz encoder sim freq 16 Mhz counts(32 * 600) ;500 cuz margin
MAX_ENC_SIM_VAL_PER_3125        .set    500

; _AX0_u16_Encsim_Freq_Flags
; -------------------------------
FREQ_FAULT_BIT                       .set    0
SKIP_FLT_CHECK_BIT                   .set    1
SKIP_ENC_SIM_DIR_BIT                 .set    2
INHIBIT_ENC_SIM_BIT                  .set    3
INHIBIT_ENC_SIM_PFB_NOT_READY_BIT    .set    4
END_INHIBIT_ENC_SIM_BIT              .set    5

ONE_SEC_DELAY                  		 .set    32000
QUARTER_SEC_DELAY              		 .set    8000

; WNS status bits
;------------------------------
WNS_RUNNING_BIT                 .set    0
WNS_FIRST_ITER_DONE             .set    1
WNS_ENDED_BIT                   .set    2
WNS_AVG_SIGN_INIT_BIT           .set    3
WNS_SIGN_BIT                    .set    4
WNS_SMOOTH_MODE_BIT             .set    5
WNS_SKIP_SEC_ITER_BIT           .set    6
WNS_TUNING_MODE_BIT             .set    7

WNS_EXCLUDE_RESET_MASK          .set   ((1 << WNS_SMOOTH_MODE_BIT)|(1 << WNS_SKIP_SEC_ITER_BIT)|(1 << WNS_TUNING_MODE_BIT))

;WNS_SMOOTH_MODE_MASK           .set   (1 << WNS_SMOOTH_MODE_BIT); RT(bit) to BG(mask) conversion
;WNS_SKIP_SEC_ITER_MASK         .set   (1 << WNS_SKIP_SEC_ITER_BIT); RT(bit) to BG(mask) conversion

;  Saturation for the WNS process (25/360*65536=4551)
WNS_MAX_ADV                     .set    011C7h

; WNS PFB direction
;----------------------------
WNS_ZERO_DIR                    .set     0
WNS_POSITIVE_DIR                .set     1
WNS_NEGATIVE_DIR                .set    -1


; _AX0_Pwm_Change_On_The_Fly
; ---------------------------
REGULAR_PWM_REQ_BIT             .set    0
REDUCE_PWM_REQ_BIT              .set    1
REDUCED_PWM_ACTIVE_BIT          .set    2

;PLL bits, set by BG
;----------------------------
PLL_ENABLED_BIT                 .set    0

;PLL bits, set by RT
;----------------------------
PLL_SYNC_OCCURRED_BIT           .set    0
PLL_LOCKED_BIT                  .set    1

; Resolver DC Offset Averaging Filter, ~0.5Hz LPF
RES_AVG_COEF_ALPHA        .set 07FF3h;
RES_AVG_COEF_BETA         .set 0000Dh;

;IU,IV,IW Filter Coefficients
;----------------------------
UVW_LPF_ALPHA                   .set    10088 ; Sampling at 16KHz, and Fc=3KHz
UVW_LPF_BETA                    .set    22680
UVW_LPF_ROUNDING                .set    16384
UVW_LPF_SHR                     .set    15

; _AX0_s16_Sensorless_Control_Bits defines
;-----------------------------------------
SL_EN_HF_INJECTION_BIT               .set      0
SL_KALMAN_BIT                        .set      1
SL_EN_BEMF_BIT                       .set      2
SL_ICMD_RATE_LIMITER_EN              .set      3
SL_EN_FORCED_COMM_BIT                .set      4
SL_STDBY_BEMF_BIT                    .set      5
SL_STDBY_KALMAN_BIT                  .set      6
SL_STDBY_HF_BIT                      .set      7
SL_HF_INIT_BIT                       .set      8
SL_HF_TO_BEMF_TRANSITION_BIT         .set      9
SL_START_KALMANING_BIT               .set      10
SL_KALMANING_STARTED_BIT             .set      11
SL_KALMAN_SYNC_BIT                   .set      12
SL_COPY_SINCOS_GAINS_BIT             .set      13
SL_INHIBIT_LOOPS_TILL_CALIB_END_BIT  .set      14

; This will lock cummutation angle and current output
LOCK_POSITION_BIT               .set      0

; _AX0_AutoTune_Bits bits defined
not_usedddddd                   .set      0
AT_AV_ACTIVE_BIT                .set      1
AT_RT_STANDSTILL_BITnotused     .set      2
AT_RT_STANDSTILL_INTEGRATE_BIT  .set      3
AT_DURING_CYCLE_BIT             .set      4
AT_PTT_VEL_START_CAPTURE_BIT    .set      5
AT_SKIP_MAX_ACC_CALC_BIT        .set      6
AT_AUTO_TRAJ_ICMD_OV_BIT        .set      7
AT_ICMD_OV_EFFORT_BIT           .set      8
AT_ICMD_OV_BIT                  .set      9
AT_NUM_EXTRM_CHNGD_BIT          .set      10
AT_ACC_PARTIAL_CYCLE_BIT        .set      11
AT_PARTIAL_CYCLE_BIT            .set      12
AT_AV_ACTIVE_2_BIT              .set      13
AT_AV_ACTIVE_3_BIT              .set      14
AT_NO_ACC_OPTIMIZE_BIT          .set      15

; _AX0_s16_Cycle_Bits defined
CYCLE_NEW_CYCLE_DATA_READY_BIT       .set      0
CYCLE_END_AND_STANDING_BIT           .set      1
CYCLE_POSSIBLE_CYCLE_FOUND_BIT       .set      2
CYCLE_FREE_BIT                       .set      3
CYCLE_POSSIBLE_STANDING_BIT          .set      4
EFFORT_EXCEEDED_BIT                  .set      5
POS_TUNE_MIN_OVERSHOOT_SCALE_BIT     .set      6
CYCLE_POSSIBLE_CYCLE_TESTED_BIT      .set      7
POS_TUNE_RT_ACTIVE_BIT               .set      8
POS_TUNE_MIN_SETTLE_TIME_SCALE_BIT   .set      9
not_usedffff_BIT                     .set      10
CYCLE_PARAM_UPDATE_BIT               .set      11
RT_TQF_EXCEEDED_BITnotused           .set      12
MOVE_SMOOTH_MODE_2_SWITCH_BIT        .set      13
PARAM_UPDATE_AT_STANDSTILL_BIT       .set      14
CYCLE_LENGTH_ERROR_BIT               .set      15

; -------------------------------------------
; ServoSense mailbox bits
; -------------------------------------------
SRVSNS_MBX_HDR_MODE_MASK            .set    3h
SRVSNS_MBX_HDR_OPERATIONAL_MODE_BIT .set    0
SRVSNS_MBX_HDR_VELOCITY_MODE_BIT    .set    1
SRVSNS_MBX_HDR_RQ_RDY_BIT           .set    2
SRVSNS_MBX_HDR_RES_RDY_BIT          .set    3
SRVSNS_MBX_HDR_RW_BIT               .set    4
SRVSNS_MBX_HDR_ERROR_BIT            .set    5
SRVSNS_MBX_HDR_ERR_BIT_MASK         .set    20h
SRVSNS_MBX_HDR_ERR_WORD_MASK        .set    01FC0h
SRVSNS_MBX_HDR_ERR_WORD_NOT_MASK    .set    0FE03h
SRVSNS_MBX_HDR_ERR_WORD_SHIFT       .set    6
SRVSNS_MBX_HDR_PRTCL_ERR_BIT        .set    13
SRVSNS_MBX_HDR_INIT_BIT             .set    14
SRVSNS_MBX_HDR_SUB_MODE_FUNC_BIT    .set    15
; -------------------------------------------


; -------------------------------------------
; ServoSense service request flags
; -------------------------------------------
SRVSNS_SRV_REQ_FLAGS_LOADING        .set    0
SRVSNS_SRV_REQ_FLAGS_COLLECT        .set    1
SRVSNS_SRV_REQ_FLAGS_GET_RESULT     .set    2

; -------------------------------------------
; ServoSense constants
; -------------------------------------------
SRVSNS_REQ_ID                       .set  5h
SRVSNS_REQ_READ_MLT_TRN_POS_FRAME1  .set  028Ah
SRVSNS_REQ_READ_MLT_TRN_POS_FRAME2  .set  0E748h
SRVSNS_REQ_READ_VELOCITY_POS_FRAME1 .set  038Ah
SRVSNS_REQ_READ_VELOCITY_POS_FRAME2 .set  0EC88h


; -------------------------------------------
; ServoSense status field
; -------------------------------------------
SRVSNS_STATUS_FIELD_FAULT_BIT    .set  0
SRVSNS_STATUS_FIELD_WARN_BIT     .set  1
SRVSNS_STATUS_FIELD_BUSY_BIT     .set  2
SRVSNS_STATUS_FIELD_ERROR_BIT    .set  3


; -------------------------------------------
; ServoSense FW download retransmit state
; -------------------------------------------
SRVSNS_FW_STATUS_PRESENT_BIT            .set  0
SRVSNS_FW_STATUS_FWDNLD_IDLE_BIT        .set  1
SRVSNS_FW_STATUS_FWDNLD_RETRANSMIT_BIT  .set  2

; AX0_s16_Crrnt_Tune_Bits bits defines 
CRRNT_TUNE_ACTIVE_BIT           .set      0
CRRNT_TUNE_CYCLE_DONE_BIT       .set      1
CRRNT_TUNE_DATA_READY_BIT       .set      2
CRRNT_TUNE_INTERNAL_BIT         .set      3
CRRNT_TUNE_OVERSHOOT_BIT        .set      4

; AX0_s16_nPWM_Crrnt_Cmnd_Bits bits defines
;------------------------------------------
NPWM_CCMD_U_POS_MASK            .set      00001h
NPWM_CCMD_V_POS_MASK            .set      00002h
NPWM_CCMD_W_POS_MASK             .set     00004h

; Defines for encoder simulation handling (AX0_u16_Encsim_ZPos_Offset_Flags)
ENCSIM_INDEX_JUST_ALIGNED_BIT             .set 0
ENCSIM_INDEX_JUST_ALIGNED_VALUE           .set 1
ENCSIM_INDEX_ALIGNED_BIT                  .set 1
ENCSIM_INDEX_BY_REAL_INDEX_BIT            .set 2
ENCSIM_INDEX_BY_ELECTANGLE_ROLLOVER_BIT   .set 3

; AX0_s16_Transition_To_Enable defines
;-------------------------------------
ASK_HW_ENABLE_BIT       .set 0
CORR_CAL_REQUIRED_BIT   .set 1
EN_REQUEST_BIT          .set 2
CURRLOOP_RDY_BIT        .set 3

;   _AX0_u16_Pos_Loop_Tsp
;--------------------------------------
TSP_250_USEC		.set 25000
TSP_125_USEC		.set 12500


; RTAT bits
;-----------------------
RTAT_BG_RESET_BIT					.set	0
RTAT_NEW_DATA_BIT					.set	1
RTAT_GAIN_LOW_BIT 					.set	2
RTAT_GAIN_HI_BIT  					.set	3
RTAT_JERK_NEW_DATA_READY_BIT		.set	4
RTAT_JERK_WAIT_FOR_HIGH_ACC_BIT 	.set	5
RTAT_JERK_WAIT_FOR_JERK_STATE_BIT 	.set	6
RTAT_JERK_STATE_BIT               	.set	7


RTAT_BG_RESET_MASK					.set	00001h
RTAT_NEW_DATA_MASK					.set	00002h
RTAT_GAIN_LOW_MASK					.set	00004h
RTAT_GAIN_HI_MASK					.set	00008h
RTAT_JERK_NEW_DATA_READY_MASK		.set	00010h
RTAT_JERK_WAIT_FOR_HIGH_ACC_MASK	.set	00020h
RTAT_JERK_WAIT_FOR_JERK_STATE_MASK 	.set	00040h
RTAT_JERK_STATE_MASK                .set	00080h 

; _AX0_u16_Dual_Loop_Control_Bits bits defines
;----------------------------------------------

DUAL_LOOP_ENABLED_BIT               .set	0
DUAL_LOOP_DESIGN_COPY_BIT           .set	1
DUAL_LOOP_NUMERICAL_PE_FAULT_BIT    .set	2
DUAL_LOOP_DIS_SFB_MONITOR_ONLY_BIT  .set    3

;_AX0_s16_DF_Run_Code bits defines
;---------------------------------
DUAL_LOOP_HOMING_INITIATE_SOFT_INDEX_BIT .set	0
DUAL_LOOP_CHANGE_PFB_RES_BIT		     .set   1
DUAL_LOOP_ENTER_ENCODER_SIMULATION_BIT   .set   2

;_AX0_u16_Use_Position_Loop bits defines
;---------------------------------
USE_POSITION_LOOP_SETTINGS_BIT	.set	0
ENTER_POS_TASK_BIT				.set    1


