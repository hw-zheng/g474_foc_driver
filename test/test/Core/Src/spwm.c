/**
  ******************************************************************************
  * @file    spwm.c
  * @brief   Closed-loop SPWM position controller
  *
  *  Architecture
  *  ============
  *  TIM17 @ 1 kHz  → SPWM_PIDCallback
  *      reads encoder, computes PID → modulation depth + direction
  *
  *  HRTIM REP @ 20 kHz → Motor_PeriodElapsedCallback → SPI-DMA encoder read
  *  SPI DMA complete   → SPWM_EncoderReadCompleteCallback
  *      current_elec_angle + 90° phase shift (±CCW) → Motor_ApplySPWM
  *
  *  CCW = positive direction.  Pole pairs = 7.
  ******************************************************************************
  */
#include "spwm.h"

/* raw ↔ degree conversion factor */
#define RAW_TO_DEG   (360.0f / 32768.0f)
#define DEG_TO_RAW   (32768.0f / 360.0f)
#define SPWM_ALIGN_MODULATION  0.3f
#define SPWM_ALIGN_SETTLE_MS   1000u
#define SPWM_DIR_HYSTERESIS    0.01f

/* ======================================================================== */
/*  SPWM_Init                                                               */
/* ======================================================================== */
void SPWM_Init(SPWM_HandleTypeDef *hspwm,
               Motor_HandleTypeDef *hmotor,
               TIM_HandleTypeDef *htim_pid)
{
    if ((hspwm == NULL) || (hmotor == NULL) || (htim_pid == NULL)) return;

    hspwm->hmotor          = hmotor;
    hspwm->htim_pid        = htim_pid;
    hspwm->target_raw      = 0u;
    hspwm->elec_offset_q31 = hmotor->elec_offset_q31; /* keep shared offset */
    hspwm->modulation      = 0.0f;
    hspwm->drive_angle_q31 = 0;
    hspwm->torque_dir      = 1;
    hspwm->enabled         = 0u;
    hspwm->aligning        = 0u;

    /* Sensible PID defaults – user should tune via SPWM_SetPID */
    hspwm->pid.kp             = 0.01f;
    hspwm->pid.ki             = 0.001f;
    hspwm->pid.kd             = 0.0f;
    hspwm->pid.integral       = 0.0f;
    hspwm->pid.prev_error     = 0.0f;
    hspwm->pid.output         = 0.0f;
    hspwm->pid.output_limit   = 1.0f;
    hspwm->pid.integral_limit = 1000.0f;
}

/* ======================================================================== */
/*  SPWM_Start – enable PID loop + PWM outputs                             */
/* ======================================================================== */
void SPWM_Start(SPWM_HandleTypeDef *hspwm)
{
    if ((hspwm == NULL) || (hspwm->hmotor == NULL) || (hspwm->htim_pid == NULL)) return;

    /* Reset PID state */
    hspwm->pid.integral   = 0.0f;
    hspwm->pid.prev_error = 0.0f;
    hspwm->pid.output     = 0.0f;
    hspwm->modulation     = 0.0f;
    hspwm->torque_dir     = 1;
    hspwm->enabled        = 1u;

    /* Keep motor-side electrical offset in sync with SPWM calibration */
    Motor_SetElecOffset(hspwm->hmotor, hspwm->elec_offset_q31);
    Motor_Start(hspwm->hmotor);
    HAL_TIM_Base_Start_IT(hspwm->htim_pid);
}

/* ======================================================================== */
/*  SPWM_Stop – disable PID loop + PWM outputs                             */
/* ======================================================================== */
void SPWM_Stop(SPWM_HandleTypeDef *hspwm)
{
    if ((hspwm == NULL) || (hspwm->hmotor == NULL) || (hspwm->htim_pid == NULL)) return;

    HAL_TIM_Base_Stop_IT(hspwm->htim_pid);
    Motor_Stop(hspwm->hmotor);
    hspwm->pid.output = 0.0f;
    hspwm->modulation = 0.0f;
    hspwm->enabled = 0u;
}

/* ======================================================================== */
/*  SPWM_SetTarget – set target mechanical angle in degrees [0, 360)       */
/* ======================================================================== */
void SPWM_SetTarget(SPWM_HandleTypeDef *hspwm, float target_deg)
{
    if (hspwm == NULL) return;

    /* Normalise into [0, 360) */
    while (target_deg >= 360.0f) target_deg -= 360.0f;
    while (target_deg <    0.0f) target_deg += 360.0f;

    hspwm->target_raw = (uint16_t)(target_deg * DEG_TO_RAW);
}

/* ======================================================================== */
/*  SPWM_SetPID                                                            */
/* ======================================================================== */
void SPWM_SetPID(SPWM_HandleTypeDef *hspwm, float kp, float ki, float kd)
{
    if (hspwm == NULL) return;

    hspwm->pid.kp = kp;
    hspwm->pid.ki = ki;
    hspwm->pid.kd = kd;
}

/* ======================================================================== */
/*  SPWM_PIDCallback – called from TIM17 update ISR (1 kHz)                */
/*  Reads latest encoder angle, computes PID → modulation + direction      */
/* ======================================================================== */
void SPWM_PIDCallback(SPWM_HandleTypeDef *hspwm)
{
    if ((hspwm == NULL) || (hspwm->hmotor == NULL) || (hspwm->hmotor->hencoder == NULL)) return;
    if (!hspwm->enabled || hspwm->aligning) return;

    uint16_t current_raw = MT6826_GetAngleRaw(hspwm->hmotor->hencoder);

    /* ---------- Circular shortest-arc error [−180°, +180°) ------------- *
     * PID operates in MECHANICAL angle raw space [0, 32767].              *
     * This is independent of pole pairs – position control only cares     *
     * about where the shaft is, not the electrical angle.                 *
     *                                                                      *
     * Shortest-arc algorithm (16384 = 32768/2):                           *
     *   target=0   current=31744 (≈350°): diff=−31744 → +32768 = 1024   *
     *     → +11.25° CCW (correct: 10° short turn)                        *
     *   target=0   current=512   (≈  5°): diff=−512, no adjust           *
     *     → −5.6° CW  (correct: 5° back)                                 *
     * Explicit int32_t casts prevent uint16 subtraction wrapping.        */
    int32_t diff = (int32_t)hspwm->target_raw - (int32_t)current_raw;
    if (diff >  16384) diff -= 32768;
    if (diff < -16384) diff += 32768;

    float error = (float)diff * RAW_TO_DEG;

    /* ---------- PID computation ---------------------------------------- */
    hspwm->pid.integral += error;
    /* Anti-windup */
    if (hspwm->pid.integral >  hspwm->pid.integral_limit)
        hspwm->pid.integral =  hspwm->pid.integral_limit;
    if (hspwm->pid.integral < -hspwm->pid.integral_limit)
        hspwm->pid.integral = -hspwm->pid.integral_limit;

    float derivative = error - hspwm->pid.prev_error;
    hspwm->pid.prev_error = error;

    float output = hspwm->pid.kp * error
                 + hspwm->pid.ki * hspwm->pid.integral
                 + hspwm->pid.kd * derivative;

    /* Clamp output */
    if (output >  hspwm->pid.output_limit) output =  hspwm->pid.output_limit;
    if (output < -hspwm->pid.output_limit) output = -hspwm->pid.output_limit;

    hspwm->pid.output = output;

    /* Keep torque-axis selection stable near zero to avoid +90/-90 phase chatter. */
    if (output > SPWM_DIR_HYSTERESIS)
    {
        hspwm->torque_dir = 1;
    }
    else if (output < -SPWM_DIR_HYSTERESIS)
    {
        hspwm->torque_dir = -1;
    }

    /* Modulation depth = |output|, clamped [0, 1] */
    float mod = (output >= 0.0f) ? output : -output;
    if (mod > 1.0f) mod = 1.0f;
    hspwm->modulation = mod;
}

/* ======================================================================== */
/*  SPWM_EncoderReadCompleteCallback – SPI-DMA complete ISR (20 kHz)       */
/*  Computes drive electrical angle with offset + 90° shift, then applies  */
/* ======================================================================== */
void SPWM_EncoderReadCompleteCallback(SPWM_HandleTypeDef *hspwm)
{
    if ((hspwm == NULL) || (hspwm->hmotor == NULL) || (hspwm->hmotor->hencoder == NULL)) return;
    if (!hspwm->enabled) return;

    int32_t angle_q31;
    float   mod;  //调制深度！！

    if (hspwm->aligning)
    {
        /* During d-axis alignment: lock at electrical 0° */
        angle_q31 = 0;
        mod       = SPWM_ALIGN_MODULATION;
    }
    else
    {
        uint16_t raw = MT6826_GetAngleRaw(hspwm->hmotor->hencoder);

        /* Convert mechanical raw → electrical angle q1.31 with offset.
         * ALL arithmetic is kept in uint32_t (mod-2^32) to exploit natural
         * circular wrapping for a 7-pole motor:
         *   raw=0..32767   → elec turns 0..7 circles in uint32 space
         *   Adding uint32 offset shifts the zero reference.
         * CORDIC accepts any int32_t bit pattern as a valid q1.31 input
         * (full [−π, π) range), so the final cast never causes overflow.  */
        int32_t elec_q31 = (int32_t)(
            (uint32_t)raw * MOTOR_ANGLE_SCALE + hspwm->elec_offset_q31
        );

        /* 90° phase shift on q-axis to produce torque:
         *   CCW (positive PID output) → +90° electrical
         *   CW  (negative PID output) → −90° electrical              */
        int32_t phase = (hspwm->torque_dir >= 0)
                  ?  SPWM_Q31_90DEG
                  : -SPWM_Q31_90DEG;

        angle_q31 = elec_q31 + phase;
        mod       = hspwm->modulation;
    }

    hspwm->drive_angle_q31 = angle_q31;
    Motor_ApplySPWM(hspwm->hmotor, angle_q31, mod);
}

/* ======================================================================== */
/*  SPWM_AlignToZero                                                       */
/*  Apply d-axis current only → motor rotates to electrical 0°             */
/*  Then record encoder offset.  Call AFTER Motor_Init, BEFORE SPWM_Start  */
/* ======================================================================== */
void SPWM_AlignToZero(SPWM_HandleTypeDef *hspwm)
{
    if ((hspwm == NULL) || (hspwm->hmotor == NULL) || (hspwm->hmotor->hencoder == NULL))
    {
        return;
    }

    /* Enter alignment mode: the 20 kHz SPI-DMA loop will continuously
       apply electrical angle = 0 at moderate voltage                      */
    hspwm->aligning = 1u;
    hspwm->enabled  = 1u;
    hspwm->pid.output = 0.0f;
    hspwm->modulation = 0.0f;
    Motor_Start(hspwm->hmotor);

    /* Wait for rotor to settle on d-axis */
    HAL_Delay(SPWM_ALIGN_SETTLE_MS);

    /* Record encoder position at electrical zero and compute offset.
     * We need: raw_at_zero * SCALE + offset ≡ 0  (mod 2^32)
     * So:      offset = 0 − raw_at_zero * SCALE   (uint32 two's complement)
     * Stored as uint32_t to avoid signed-negation UB (e.g. −INT32_MIN).  */
    uint16_t raw = MT6826_GetAngleRaw(hspwm->hmotor->hencoder);
    hspwm->elec_offset_q31 = 0u - (uint32_t)raw * MOTOR_ANGLE_SCALE;
    Motor_SetElecOffset(hspwm->hmotor, hspwm->elec_offset_q31);

    /* Exit alignment, stop outputs until user calls SPWM_Start */
    Motor_Stop(hspwm->hmotor);
    hspwm->aligning = 0u;
    hspwm->enabled  = 0u;
}
