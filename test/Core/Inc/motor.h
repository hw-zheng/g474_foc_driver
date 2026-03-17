/**
  ******************************************************************************
  * @file    motor.h
  * @brief   SPWM brushless motor driver (HRTIM + CORDIC + MT6826S encoder)
  ******************************************************************************
  */
#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "mt6826.h"

/* --------------- Motor parameters ---------------------------------------- */
#define MOTOR_POLE_PAIRS      7u          /* 7 pole-pair motor               */
#define MOTOR_HRTIM_PERIOD    34000u      /* Timer A/B/E period (MUL4→20kHz) */
#define MOTOR_PWM_FREQ_HZ     20000u      /* electrical update period budget */

/* raw_15bit → electrical angle q1.31:
 *   scale = pole_pairs × 2^32 / 32768 = 7 × 131072 = 917504
 *   uint32 overflow provides natural circular wrapping.               */
#define MOTOR_ANGLE_SCALE     917504u

/* Safe compare-value limits (dead-time guard band) */
#define MOTOR_CMP_MIN         96u
#define MOTOR_CMP_MAX         (MOTOR_HRTIM_PERIOD - 96u)

/* 1: Va,Vb,Vc = [sin(a), sin(a-120), sin(a+120)] (default)
 * 0: swap B/C phase order if hardware wiring uses opposite sequence. */
#define MOTOR_PHASE_SEQUENCE_ABC      0u

typedef struct {
  uint32_t repetition_timer_idx;
  uint32_t phase_timer_idx[3];
  uint32_t output_mask;
  uint32_t counter_mask;
} Motor_TimerConfigTypeDef;

/* --------------- Handle -------------------------------------------------- */
typedef struct {
    HRTIM_HandleTypeDef   *hhrtim;
    CORDIC_HandleTypeDef  *hcordic;
    MT6826_HandleTypeDef  *hencoder;
  Motor_TimerConfigTypeDef timing;
    float                  modulation_index;   /* 0.0 ~ 1.0                 */
    uint32_t               elec_offset_q31;    /* electrical zero offset     */
    uint16_t               cmp_a;              /* current Timer A CMP1      */
    uint16_t               cmp_b;              /* current Timer B CMP1      */
    uint16_t               cmp_c;              /* current C-phase CMP1 (E)  */
    uint32_t               apply_cycles_last;  /* calc->CMP write elapsed    */
    uint32_t               apply_cycles_max;   /* max elapsed cycles         */
    uint32_t               apply_overrun_count;/* elapsed > one PWM period   */
    uint32_t               apply_budget_cycles;/* SystemCoreClock/PWM_FREQ   */
    volatile uint8_t       running;
} Motor_HandleTypeDef;

/* --------------- API ----------------------------------------------------- */
void  Motor_Init(Motor_HandleTypeDef *hmotor,
                 HRTIM_HandleTypeDef *hhrtim,
                 CORDIC_HandleTypeDef *hcordic,
                 MT6826_HandleTypeDef *hencoder,
                 const Motor_TimerConfigTypeDef *timing);
void  Motor_Start(Motor_HandleTypeDef *hmotor);
void  Motor_Stop(Motor_HandleTypeDef *hmotor);
void  Motor_SetModulation(Motor_HandleTypeDef *hmotor, float mod);
void  Motor_SetElecOffset(Motor_HandleTypeDef *hmotor, uint32_t offset_q31);

/* Called from the configured HRTIM repetition interrupt */
void  Motor_PeriodElapsedCallback(Motor_HandleTypeDef *hmotor);

/* Called from SPI-DMA transfer-complete interrupt */
void  Motor_EncoderReadCompleteCallback(Motor_HandleTypeDef *hmotor);

/* Apply SPWM at a given q1.31 electrical angle and modulation depth [0,1] */
void  Motor_ApplySPWM(Motor_HandleTypeDef *hmotor, int32_t elec_angle_q31, float modulation);

#endif /* MOTOR_H */
