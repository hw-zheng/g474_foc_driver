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

/* ---------- Timing monitor (DWT cycle counter) -------------------------- */
#if defined(DWT) && defined(DWT_CTRL_CYCCNTENA_Msk) && \
    defined(CoreDebug) && defined(CoreDebug_DEMCR_TRCENA_Msk)
#define MOTOR_DWT_AVAILABLE 1u
#else
#define MOTOR_DWT_AVAILABLE 0u
#endif

static inline void Motor_EnableCycleCounter(void)
{
#if (MOTOR_DWT_AVAILABLE == 1u)
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0u;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
#endif
}

static inline uint32_t Motor_ReadCycleCounter(void)
{
#if (MOTOR_DWT_AVAILABLE == 1u)
    return DWT->CYCCNT;
#else
    return 0u;
#endif
}

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
                MT6826_HandleTypeDef *hencoder,
                const Motor_TimerConfigTypeDef *timing)
{
    uint32_t phase_idx;

    if ((hmotor == NULL) || (hhrtim == NULL) || (hcordic == NULL) ||
        (hencoder == NULL) || (timing == NULL))
    {
        return;
    }

    hmotor->hhrtim   = hhrtim;
    hmotor->hcordic  = hcordic;
    hmotor->hencoder = hencoder;
    hmotor->timing   = *timing;
    hmotor->modulation_index = 0.0f;
    hmotor->elec_offset_q31 = 0u;
    hmotor->cmp_a    = MOTOR_HRTIM_PERIOD >> 1;   /* 50 % initial duty */
    hmotor->cmp_b    = MOTOR_HRTIM_PERIOD >> 1;
    hmotor->cmp_c    = MOTOR_HRTIM_PERIOD >> 1;
    hmotor->apply_cycles_last = 0u;
    hmotor->apply_cycles_max = 0u;
    hmotor->apply_overrun_count = 0u;
    hmotor->apply_budget_cycles = SystemCoreClock / MOTOR_PWM_FREQ_HZ;
    if (hmotor->apply_budget_cycles == 0u) hmotor->apply_budget_cycles = 1u;
    hmotor->running  = 0u;

    Motor_EnableCycleCounter();

    for (phase_idx = 0u; phase_idx < 3u; phase_idx++)
    {
        SET_BIT(hhrtim->Instance->sTimerxRegs[hmotor->timing.phase_timer_idx[phase_idx]].TIMxCR,
                HRTIM_TIMCR_PREEN);
    }

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
    hhrtim->Instance->sTimerxRegs[hmotor->timing.phase_timer_idx[0]].CMP1xR = hmotor->cmp_a;
    hhrtim->Instance->sTimerxRegs[hmotor->timing.phase_timer_idx[1]].CMP1xR = hmotor->cmp_b;
    hhrtim->Instance->sTimerxRegs[hmotor->timing.phase_timer_idx[2]].CMP1xR = hmotor->cmp_c;
}

/* ======================================================================== */
/*  Motor_Start – enable PWM outputs and repetition interrupt               */
/* ======================================================================== */
void Motor_Start(Motor_HandleTypeDef *hmotor)
{
    hmotor->running = 1u;

    /* Enable repetition interrupt on the configured sampling timer */
    __HAL_HRTIM_TIMER_ENABLE_IT(hmotor->hhrtim, hmotor->timing.repetition_timer_idx,
                                 HRTIM_TIM_IT_REP);

    HAL_HRTIM_WaveformOutputStart(hmotor->hhrtim, hmotor->timing.output_mask);

    HAL_HRTIM_WaveformCountStart(hmotor->hhrtim, hmotor->timing.counter_mask);
}

/* ======================================================================== */
/*  Motor_Stop – disable outputs and interrupt                              */
/* ======================================================================== */
void Motor_Stop(Motor_HandleTypeDef *hmotor)
{
    hmotor->running = 0u;

    __HAL_HRTIM_TIMER_DISABLE_IT(hmotor->hhrtim, hmotor->timing.repetition_timer_idx,
                                  HRTIM_TIM_IT_REP);

    HAL_HRTIM_WaveformOutputStop(hmotor->hhrtim, hmotor->timing.output_mask);

    HAL_HRTIM_WaveformCountStop(hmotor->hhrtim, hmotor->timing.counter_mask);
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
/*  Motor_SetElecOffset                                                     */
/* ======================================================================== */
void Motor_SetElecOffset(Motor_HandleTypeDef *hmotor, uint32_t offset_q31)
{
    hmotor->elec_offset_q31 = offset_q31;
}

/* ======================================================================== */
/*  Motor_PeriodElapsedCallback                                             */
/*  Called from configured HRTIM repetition ISR → triggers encoder read      */
/* ======================================================================== */
void Motor_PeriodElapsedCallback(Motor_HandleTypeDef *hmotor)
{
    if (!hmotor->running) return;
    if (!hmotor->hencoder->dma_done) return;   /* previous read still pending */
    MT6826_StartDMARead(hmotor->hencoder);
}

/* ======================================================================== */
/*  Motor_ApplySPWM – reusable CORDIC→CMP core                             */
/*  Compute 3-phase compare values from a given elec angle and modulation   */
/* ======================================================================== */
void Motor_ApplySPWM(Motor_HandleTypeDef *hmotor, int32_t elec_angle_q31, float modulation)
{
    uint32_t start_cycles = Motor_ReadCycleCounter();

    if (modulation < 0.0f) modulation = 0.0f;
    if (modulation > 1.0f) modulation = 1.0f;

    /* CORDIC hardware-accelerated sin/cos (polling, ≈6 cycles) */
    WRITE_REG(hmotor->hcordic->Instance->WDATA, (uint32_t)elec_angle_q31);
    while (__HAL_CORDIC_GET_FLAG(hmotor->hcordic, CORDIC_FLAG_RRDY) == 0U) {}
    int32_t sin_q31 = (int32_t)READ_REG(hmotor->hcordic->Instance->RDATA);
    int32_t cos_q31 = (int32_t)READ_REG(hmotor->hcordic->Instance->RDATA);

    float sin_f = (float)sin_q31 * Q31_TO_FLOAT;
    float cos_f = (float)cos_q31 * Q31_TO_FLOAT;

    /*  Va = sin(θe)
     *  Vb = sin(θe − 120°) = −0.5·sin − (√3/2)·cos
     *  Vc = sin(θe + 120°) = −0.5·sin + (√3/2)·cos
     *  CMP = Period/2 × (1 + m · Vx)                                      */
    float half = (float)(MOTOR_HRTIM_PERIOD >> 1);

    float va = sin_f;
#if (MOTOR_PHASE_SEQUENCE_ABC == 1u)
    float vb = -0.5f * sin_f - SQRT3_OVER_2 * cos_f;
    float vc = -0.5f * sin_f + SQRT3_OVER_2 * cos_f;
#else
    float vb = -0.5f * sin_f + SQRT3_OVER_2 * cos_f;
    float vc = -0.5f * sin_f - SQRT3_OVER_2 * cos_f;
#endif

    int32_t ca = (int32_t)(half + half * modulation * va);
    int32_t cb = (int32_t)(half + half * modulation * vb);
    int32_t cc = (int32_t)(half + half * modulation * vc);

    hmotor->cmp_a = Motor_ClampCmp(ca);
    hmotor->cmp_b = Motor_ClampCmp(cb);
    hmotor->cmp_c = Motor_ClampCmp(cc);

    hmotor->hhrtim->Instance->sTimerxRegs[hmotor->timing.phase_timer_idx[0]].CMP1xR = hmotor->cmp_a;
    hmotor->hhrtim->Instance->sTimerxRegs[hmotor->timing.phase_timer_idx[1]].CMP1xR = hmotor->cmp_b;
    hmotor->hhrtim->Instance->sTimerxRegs[hmotor->timing.phase_timer_idx[2]].CMP1xR = hmotor->cmp_c;

    uint32_t elapsed = Motor_ReadCycleCounter() - start_cycles;
    hmotor->apply_cycles_last = elapsed;
    if (elapsed > hmotor->apply_cycles_max) hmotor->apply_cycles_max = elapsed;
    if (elapsed > hmotor->apply_budget_cycles) hmotor->apply_overrun_count++;
}

/* ======================================================================== */
/*  Motor_EncoderReadCompleteCallback                                       */
/*  Called from SPI-DMA complete ISR → CORDIC → update HRTIM CMP           */
/* ======================================================================== */
void Motor_EncoderReadCompleteCallback(Motor_HandleTypeDef *hmotor)
{
    if (!hmotor->running) return;

    uint16_t raw = MT6826_GetAngleRaw(hmotor->hencoder);
    int32_t elec_q31 = (int32_t)(
        (uint32_t)raw * MOTOR_ANGLE_SCALE + hmotor->elec_offset_q31
    );

    Motor_ApplySPWM(hmotor, elec_q31, hmotor->modulation_index);
}
