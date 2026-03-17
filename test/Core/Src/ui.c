/**
  ******************************************************************************
  * @file    ui.c
  * @brief   Rotary-encoder + push-button PID tuner with VOFA+ FireWater stream
  *
  *  FireWater protocol (VOFA+) — text/CSV format
  *  -----------------------------------------------
  *  Format: "ch0,ch1,...,chN\n"
  *  Every TIM8 tick (throttled by DMA bandwidth):
  *    snprintf → "angle,kp,ki,kd,param_idx\n"
  *  e.g.: "123.46,0.0200,0.000500,0.000000,0\n"
  *
  *  DMA arbitration
  *  ---------------
  *  tx_busy flag prevents concurrent DMA launches.  If the previous
  *  frame has not finished, the current tick is silently skipped.
  ******************************************************************************
  */
#include "ui.h"
#include <string.h>
#include <stdio.h>

/* =========================================================================
 * UI_Init
 * ========================================================================= */
void UI_Init(UI_HandleTypeDef  *hui,
             SPWM_HandleTypeDef *hspwm,
             UART_HandleTypeDef *huart,
             TIM_HandleTypeDef  *htim_enc,
             TIM_HandleTypeDef  *htim_tick)
{
    hui->hspwm          = hspwm;
    hui->huart          = huart;
    hui->htim_enc       = htim_enc;

    hui->param_idx      = UI_PARAM_KP;
    hui->enc_last       = (int16_t)__HAL_TIM_GET_COUNTER(htim_enc);

    hui->tx_busy        = 0u;

    hui->btn_time       = 0u;
    hui->btn_state      = 1u;           /* PULLUP idle state = logic 1      */

    /* Start TIM3 encoder counting */
    HAL_TIM_Encoder_Start(htim_enc, TIM_CHANNEL_ALL);

    /* TIM8_UP_IRQn NVIC is configured by CubeMX via HAL_TIM_Base_MspInit
     * (priority 10, sub-priority 0).  Only need to start the timer here. */
    HAL_TIM_Base_Start_IT(htim_tick);
}

/* =========================================================================
 * UI_Poll – non-blocking, call from main loop
 * ========================================================================= */
void UI_Poll(UI_HandleTypeDef *hui)
{
    /* -------------------------------------------------------------------- *
     * Encoder delta → PID parameter adjustment                              *
     * TIM3 is 16-bit; casting CNT to int16_t gives signed wrap-around      *
     * arithmetic that correctly handles the 0 ↔ 65535 boundary.            *
     * -------------------------------------------------------------------- */
    int16_t enc_now = (int16_t)__HAL_TIM_GET_COUNTER(hui->htim_enc);
    int16_t delta   = enc_now - hui->enc_last;
    hui->enc_last   = enc_now;

    if (delta != 0)
    {
        float kp = hui->hspwm->pid.kp;
        float ki = hui->hspwm->pid.ki;
        float kd = hui->hspwm->pid.kd;

        switch (hui->param_idx)
        {
            case UI_PARAM_KP:
                kp += (float)delta * UI_STEP_KP;
                if (kp < 0.0f) kp = 0.0f;
                break;

            case UI_PARAM_KI:
                ki += (float)delta * UI_STEP_KI;
                if (ki < 0.0f) ki = 0.0f;
                break;

            default:  /* UI_PARAM_KD */
                kd += (float)delta * UI_STEP_KD;
                if (kd < 0.0f) kd = 0.0f;
                break;
        }

        /* SPWM_SetPID only updates kp/ki/kd, does NOT reset the integrator */
        SPWM_SetPID(hui->hspwm, kp, ki, kd);
    }

    /* -------------------------------------------------------------------- *
     * Button PB7 debounce (active-low, PULLUP)                              *
     * Falling edge = button pressed → cycle to next parameter               *
     * -------------------------------------------------------------------- */
    uint8_t raw = (GPIOB->IDR & GPIO_PIN_7) ? 1u : 0u;
    if (raw != hui->btn_state)
    {
        uint32_t now = HAL_GetTick();
        if ((now - hui->btn_time) >= UI_BTN_DEBOUNCE_MS)
        {
            hui->btn_state = raw;
            hui->btn_time  = now;
            if (raw == 0u)  /* falling edge = press */
            {
                hui->param_idx = (uint8_t)(
                    (hui->param_idx + 1u) % (uint8_t)UI_PARAM_COUNT);
            }
        }
    }
}

/* =========================================================================
 * UI_TIM8Callback – call from HAL_TIM_PeriodElapsedCallback (TIM8, ≈10.6 kHz)
 *
 *  Every tick: format and send FireWater CSV frame via UART DMA
 *  Frame: "angle,kp,ki,kd,param_idx\n"
 * ========================================================================= */
void UI_TIM8Callback(UI_HandleTypeDef *hui)
{
    if (hui->tx_busy)
        return;

    /* Open-loop test may skip SPWM_Init; guard against null chain. */
    if ((hui->hspwm == NULL) ||
        (hui->hspwm->hmotor == NULL) ||
        (hui->hspwm->hmotor->hencoder == NULL))
    {
        return;
    }

    float deg = (float)MT6826_GetAngleRaw(hui->hspwm->hmotor->hencoder)
                * (360.0f / 32768.0f);

    int len = snprintf((char *)hui->fw_buf, UI_FW_BUF_SIZE,
                       "%.2f,%.4f,%.6f,%.6f,%d\n",
                       (double)deg,
                       (double)hui->hspwm->pid.kp,
                       (double)hui->hspwm->pid.ki,
                       (double)hui->hspwm->pid.kd,
                       (int)hui->param_idx);
    if (len <= 0)
        return;

    hui->tx_busy = 1u;
    if (HAL_UART_Transmit_DMA(hui->huart,
                              hui->fw_buf,
                              (uint16_t)len) != HAL_OK)
    {
        hui->tx_busy = 0u;
    }
}

/* =========================================================================
 * UI_TxCpltCallback – call from HAL_UART_TxCpltCallback
 * ========================================================================= */
void UI_TxCpltCallback(UI_HandleTypeDef *hui)
{
    hui->tx_busy = 0u;
}
