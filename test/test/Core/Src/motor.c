/**
  ******************************************************************************
  * @file    motor.c
  * @brief   SPWM brushless motor driver (HRTIM + CORDIC + MT6826S encoder)
  *
  *  Control loop (runs every HRTIM period ≈ 50 µs / 20 kHz):
  *    1. HRTIM Timer-A REP interrupt  →  Motor_PeriodElapsedCallback()
  *       → starts SPI-DMA read of MT6826S angle
  *    2. SPI DMA complete interrupt   →  Motor_EncoderReadCompleteCallback()
  *       → CORDIC sin/cos  → compute 3-phase compare values
  *       → write CMP1 preload registers (applied at next period)
  ******************************************************************************
  */
#include "motor.h"

/* ---------- Constants ---------------------------------------------------- */
#define SQRT3_OVER_2   0.86602540378f
#define Q31_TO_FLOAT   (1.0f / 2147483648.0f)

/* ---------- Helper ------------------------------------------------------- */
static inline uint16_t Motor_ClampCmp(int32_t val)
{
    if (val < (int32_t)MOTOR_CMP_MIN) return MOTOR_CMP_MIN;
    if (val > (int32_t)MOTOR_CMP_MAX) return MOTOR_CMP_MAX;
    return (uint16_t)val;
}

/* ======================================================================== */
/*  Motor_Init                                                              */
/* ======================================================================== */
void Motor_Init(Motor_HandleTypeDef *hmotor,
                HRTIM_HandleTypeDef *hhrtim,
                CORDIC_HandleTypeDef *hcordic,
                MT6826_HandleTypeDef *hencoder)
{
    hmotor->hhrtim   = hhrtim;
    hmotor->hcordic  = hcordic;
    hmotor->hencoder = hencoder;
    hmotor->modulation_index = 0.0f;
    hmotor->cmp_a    = MOTOR_HRTIM_PERIOD >> 1;   /* 50 % initial duty */
    hmotor->cmp_b    = MOTOR_HRTIM_PERIOD >> 1;
    hmotor->cmp_c    = MOTOR_HRTIM_PERIOD >> 1;
    hmotor->running  = 0u;

    /* Ensure Timer C preload is enabled (CubeMX may leave it disabled) */
    SET_BIT(hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].TIMxCR,
            HRTIM_TIMCR_PREEN);

    /* Configure CORDIC once: sine function, q1.31, 1-write / 2-read */
    CORDIC_ConfigTypeDef cfg = {0};
    cfg.Function  = CORDIC_FUNCTION_SINE;
    cfg.Scale     = CORDIC_SCALE_0;
    cfg.InSize    = CORDIC_INSIZE_32BITS;
    cfg.OutSize   = CORDIC_OUTSIZE_32BITS;
    cfg.NbWrite   = CORDIC_NBWRITE_1;    /* angle only, modulus = 1 */
    cfg.NbRead    = CORDIC_NBREAD_2;     /* sin + cos              */
    cfg.Precision = CORDIC_PRECISION_6CYCLES;
    HAL_CORDIC_Configure(hcordic, &cfg);

    /* Write initial 50 % compare values */
    hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = hmotor->cmp_a;
    hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = hmotor->cmp_b;
    hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = hmotor->cmp_c;
}

/* ======================================================================== */
/*  Motor_Start – enable PWM outputs and repetition interrupt               */
/* ======================================================================== */
void Motor_Start(Motor_HandleTypeDef *hmotor)
{
    hmotor->running = 1u;

    /* Enable repetition interrupt on Timer A */
    __HAL_HRTIM_TIMER_ENABLE_IT(hmotor->hhrtim, HRTIM_TIMERINDEX_TIMER_A,
                                 HRTIM_TIM_IT_REP);

    /* Turn on six complementary outputs (TA1/TA2, TB1/TB2, TC1/TC2) */
    HAL_HRTIM_WaveformOutputStart(hmotor->hhrtim,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

    /* Start Timer A / B / C counters */
    HAL_HRTIM_WaveformCountStart(hmotor->hhrtim,
        HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_C);
}

/* ======================================================================== */
/*  Motor_Stop – disable outputs and interrupt                              */
/* ======================================================================== */
void Motor_Stop(Motor_HandleTypeDef *hmotor)
{
    hmotor->running = 0u;

    __HAL_HRTIM_TIMER_DISABLE_IT(hmotor->hhrtim, HRTIM_TIMERINDEX_TIMER_A,
                                  HRTIM_TIM_IT_REP);

    HAL_HRTIM_WaveformOutputStop(hmotor->hhrtim,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

    HAL_HRTIM_WaveformCountStop(hmotor->hhrtim,
        HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_C);
}

/* ======================================================================== */
/*  Motor_SetModulation                                                     */
/* ======================================================================== */
void Motor_SetModulation(Motor_HandleTypeDef *hmotor, float mod)
{
    if (mod < 0.0f) mod = 0.0f;
    if (mod > 1.0f) mod = 1.0f;
    hmotor->modulation_index = mod;
}

/* ======================================================================== */
/*  Motor_PeriodElapsedCallback                                             */
/*  Called from HRTIM Timer-A repetition ISR → triggers encoder read        */
/* ======================================================================== */
void Motor_PeriodElapsedCallback(Motor_HandleTypeDef *hmotor)
{
    if (!hmotor->running) return;
    if (!hmotor->hencoder->dma_done) return;   /* previous read still pending */
    MT6826_StartDMARead(hmotor->hencoder);
}

/* ======================================================================== */
/*  Motor_EncoderReadCompleteCallback                                       */
/*  Called from SPI-DMA complete ISR → CORDIC → update HRTIM CMP           */
/* ======================================================================== */
void Motor_EncoderReadCompleteCallback(Motor_HandleTypeDef *hmotor)
{
    if (!hmotor->running) return;

    /* ---- 1. Read mechanical angle from encoder ------------------------- */
    uint16_t raw = MT6826_GetAngleRaw(hmotor->hencoder);

    /* ---- 2. Mechanical → electrical angle (q1.31, circular) ------------ */
    /*   elec_q31 = raw × POLE_PAIRS × 2^32 / 32768                       */
    /*   uint32 multiplication naturally wraps → circular angle            */
    int32_t elec_q31 = (int32_t)((uint32_t)raw * MOTOR_ANGLE_SCALE);

    /* ---- 3. CORDIC hardware-accelerated sin/cos (polling, ≈6 cycles) --- */
    WRITE_REG(hmotor->hcordic->Instance->WDATA, (uint32_t)elec_q31);
    while (__HAL_CORDIC_GET_FLAG(hmotor->hcordic, CORDIC_FLAG_RRDY) == 0U) {}
    int32_t sin_q31 = (int32_t)READ_REG(hmotor->hcordic->Instance->RDATA);
    int32_t cos_q31 = (int32_t)READ_REG(hmotor->hcordic->Instance->RDATA);

    /* ---- 4. q1.31 → float --------------------------------------------- */
    float sin_f = (float)sin_q31 * Q31_TO_FLOAT;
    float cos_f = (float)cos_q31 * Q31_TO_FLOAT;

    /* ---- 5. Three-phase SPWM ------------------------------------------- *
     *   Va = sin(θe)
     *   Vb = sin(θe − 120°) = −0.5·sin − (√3/2)·cos
     *   Vc = sin(θe + 120°) = −0.5·sin + (√3/2)·cos
     *
     *   CMP = Period/2 × (1 + m · Vx)     where m = modulation index      */
    float m    = hmotor->modulation_index;
    float half = (float)(MOTOR_HRTIM_PERIOD >> 1);

    float va = sin_f;
    float vb = -0.5f * sin_f - SQRT3_OVER_2 * cos_f;
    float vc = -0.5f * sin_f + SQRT3_OVER_2 * cos_f;

    int32_t ca = (int32_t)(half + half * m * va);
    int32_t cb = (int32_t)(half + half * m * vb);
    int32_t cc = (int32_t)(half + half * m * vc);

    hmotor->cmp_a = Motor_ClampCmp(ca);
    hmotor->cmp_b = Motor_ClampCmp(cb);
    hmotor->cmp_c = Motor_ClampCmp(cc);

    /* ---- 6. Write CMP1 preload registers (shadow → active at next period) */
    hmotor->hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = hmotor->cmp_a;
    hmotor->hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = hmotor->cmp_b;
    hmotor->hhrtim->Instance->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = hmotor->cmp_c;
}
