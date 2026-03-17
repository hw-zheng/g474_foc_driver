/**
  ******************************************************************************
  * @file    spwm.h
  * @brief   Closed-loop SPWM position controller
  *          PID position loop (TIM17 @ 1 kHz) + HRTIM SPWM (20 kHz)
  ******************************************************************************
  */
#ifndef SPWM_H
#define SPWM_H

#include "motor.h"

/* 90° in CORDIC q1.31 : [-1,1) → [-π,π),  90° = π/2 → 0.5 → 0x40000000 */
#define SPWM_Q31_90DEG  ((int32_t)0x40000000)

/* --------------- PID ----------------------------------------------------- */
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float prev_error;
    float output;             /* signed: >0 CCW, <0 CW              */
    float output_limit;       /* |output| clamp, default 1.0         */
    float integral_limit;     /* anti-windup on |integral|            */
} SPWM_PID_TypeDef;

/* --------------- Handle -------------------------------------------------- */
typedef struct {
    Motor_HandleTypeDef  *hmotor;
    TIM_HandleTypeDef    *htim_pid;         /* TIM17 for 1 kHz PID loop      */
    SPWM_PID_TypeDef      pid;
    uint16_t              target_raw;       /* target mech-angle [0,32767]   */
    /* Calibration offset stored as uint32_t:
     * elec_q31 = (uint32_t)(raw * MOTOR_ANGLE_SCALE) + elec_offset_q31
     * All arithmetic stays in uint32 (mod-2^32) → natural circular wrap.
     * After AlignToZero: offset = 0 − raw_at_zero * MOTOR_ANGLE_SCALE        */
    uint32_t              elec_offset_q31;  /* encoder→elec calibration offset */
    float                 modulation;       /* |PID output| → mod depth       */
    int32_t               drive_angle_q31;  /* latest drive elec angle (debug) */
    int8_t                torque_dir;       /* +1 CCW torque, -1 CW torque    */ //方向滞回
    volatile uint8_t      enabled;
    volatile uint8_t      aligning;         /* d-axis alignment in progress   */
} SPWM_HandleTypeDef;

/* --------------- API ----------------------------------------------------- */
void  SPWM_Init(SPWM_HandleTypeDef *hspwm,
                Motor_HandleTypeDef *hmotor,
                TIM_HandleTypeDef *htim_pid);
void  SPWM_Start(SPWM_HandleTypeDef *hspwm);
void  SPWM_Stop(SPWM_HandleTypeDef *hspwm);
void  SPWM_SetTarget(SPWM_HandleTypeDef *hspwm, float target_deg);
void  SPWM_SetPID(SPWM_HandleTypeDef *hspwm, float kp, float ki, float kd);

/* Called from TIM17 period-elapsed ISR (1 kHz) */
void  SPWM_PIDCallback(SPWM_HandleTypeDef *hspwm);

/* Called from SPI-DMA complete ISR (20 kHz) – replaces Motor callback */
void  SPWM_EncoderReadCompleteCallback(SPWM_HandleTypeDef *hspwm);

/* Apply d-axis only current → rotate motor to electrical 0° and calibrate
 * encoder offset.  Call AFTER Motor_Init, BEFORE SPWM_Start.              */
void  SPWM_AlignToZero(SPWM_HandleTypeDef *hspwm);

#endif /* SPWM_H */
