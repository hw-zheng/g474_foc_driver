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
#define MOTOR_HRTIM_PERIOD    34000u      /* Timer A/B/C period (MUL4→20kHz) */

/* raw_15bit → electrical angle q1.31:
 *   scale = pole_pairs × 2^32 / 32768 = 7 × 131072 = 917504
 *   uint32 overflow provides natural circular wrapping.               */
#define MOTOR_ANGLE_SCALE     917504u

/* Safe compare-value limits (dead-time guard band) */
#define MOTOR_CMP_MIN         96u
#define MOTOR_CMP_MAX         (MOTOR_HRTIM_PERIOD - 96u)

/* --------------- Handle -------------------------------------------------- */
typedef struct {
    HRTIM_HandleTypeDef   *hhrtim;
    CORDIC_HandleTypeDef  *hcordic;
    MT6826_HandleTypeDef  *hencoder;
    float                  modulation_index;   /* 0.0 ~ 1.0                 */
    uint16_t               cmp_a;              /* current Timer A CMP1      */
    uint16_t               cmp_b;              /* current Timer B CMP1      */
    uint16_t               cmp_c;              /* current Timer C CMP1      */
    volatile uint8_t       running;
} Motor_HandleTypeDef;

/* --------------- API ----------------------------------------------------- */
void  Motor_Init(Motor_HandleTypeDef *hmotor,
                 HRTIM_HandleTypeDef *hhrtim,
                 CORDIC_HandleTypeDef *hcordic,
                 MT6826_HandleTypeDef *hencoder);
void  Motor_Start(Motor_HandleTypeDef *hmotor);
void  Motor_Stop(Motor_HandleTypeDef *hmotor);
void  Motor_SetModulation(Motor_HandleTypeDef *hmotor, float mod);

/* Called from HRTIM Timer-A repetition interrupt */
void  Motor_PeriodElapsedCallback(Motor_HandleTypeDef *hmotor);

/* Called from SPI-DMA transfer-complete interrupt */
void  Motor_EncoderReadCompleteCallback(Motor_HandleTypeDef *hmotor);

#endif /* MOTOR_H */
