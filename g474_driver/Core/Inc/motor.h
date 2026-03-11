#ifndef __MOTOR_H
#define __MOTOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <math.h>


#define M_PI 3.14


/* ============================================================
 * HRTIM / PWM 参数
 * 上升计数 (Up-count)：Set=TIMPER，Reset=TIMCMP1
 * fHRTIM = 170MHz × 8 = 1.36GHz
 * PWM频率 = 20kHz  →  Period = 1360000000 / 20000 = 68000
 * 占空比  = CMP1 / Period，50%对应 CMP1=34000
 * ============================================================ */
#define HRTIM_PWM_PERIOD        (34000U)        /* HRTIM计数周期               */
#define HRTIM_PWM_HALF_PERIOD   (17000U)        /* 半周期，对应 50% 占空比     */
#define MOTOR_PWM_FREQ_HZ       (20000U)        /* PWM载波频率              */
#define MOTOR_SINE_FREQ_HZ      (50U)           /* 基波频率 (可运行时修改)  */

/* 正弦表长度（每个电气周期的采样点数）
 * 更新率 = 20000Hz, 基波50Hz → 每周期400步 */
#define SINE_TABLE_SIZE         (400U)

/* 调制深度 0.0~1.0，1.0 = 满幅SPWM */
#define MOTOR_MODULATION_INDEX  (0.85f)

/* 死区时间 (ns) */
#define MOTOR_DEAD_TIME_NS      (200U)

/* ============================================================
 * DRV8323 引脚定义
 * SPI1: PA5=SCK, PA6=MISO, PA7=MOSI
 *       PA3=NSS1 (软件片选), PA4=NSS2 (备用)
 * PA2 = DRV_ENABLE  (HIGH=工作, LOW=待机)
 * PA12= FAULT1/nFAULT (开漏输出, 低有效, 需上拉)
 * ============================================================ */
#define DRV_SPI_NSS_PORT        GPIOA
#define DRV_SPI_NSS1_PIN        GPIO_PIN_3      /* PA3 SPI1_NSS1 - DRV8323 #1 CS */
#define DRV_SPI_NSS2_PIN        GPIO_PIN_4      /* PA4 SPI1_NSS2 - DRV8323 #2 CS */

#define DRV_ENABLE_PORT         GPIOA
#define DRV_ENABLE_PIN          GPIO_PIN_2      /* PA2  HIGH=使能所有栖极驱动 */

#define DRV_NFAULT_PORT         GPIOA
#define DRV_NFAULT_PIN          GPIO_PIN_12     /* PA12 FAULT1, 开漏输出, LOW=故障 */

/* ============================================================
 * DRV8323 寄存器地址 (4-bit, 0x00~0x08)
 * ============================================================ */
#define DRV8323_REG_FAULT_STATUS1   (0x00U)  /* Fault Status Register 1     (RO) */
#define DRV8323_REG_FAULT_STATUS2   (0x01U)  /* Fault Status Register 2     (RO) */
#define DRV8323_REG_VGS_STATUS1     (0x02U)  /* VGS Status Register 1       (RO) */
#define DRV8323_REG_VGS_STATUS2     (0x03U)  /* VGS Status Register 2       (RO) */
#define DRV8323_REG_DRIVER_CTRL     (0x04U)  /* Driver Control              (RW) */
#define DRV8323_REG_GATE_DRIVE_HS   (0x05U)  /* Gate Drive HS               (RW) */
#define DRV8323_REG_GATE_DRIVE_LS   (0x06U)  /* Gate Drive LS               (RW) */
#define DRV8323_REG_OCP_CTRL        (0x07U)  /* OCP Control                 (RW) */
#define DRV8323_REG_CSA_CTRL        (0x08U)  /* CSA Control                 (RW) */

/* ============================================================
 * DRV8323 寄存器位域宏
 * ============================================================ */

/* --- 0x04 Driver Control --- */
#define DRV_DRV_CTRL_CLR_FLT        (1U << 10)  /* 清除锁存故障 (自清除位)         */
#define DRV_DRV_CTRL_BRAKE          (1U << 9)   /* 制动模式                         */
#define DRV_DRV_CTRL_COAST          (1U << 8)   /* 滑行: 所有输出 Hi-Z               */
#define DRV_DRV_CTRL_PWM_MODE_6X    (0U << 4)   /* 6x-PWM：HRTIM 6路独立控制 (默认) */
#define DRV_DRV_CTRL_PWM_MODE_3X    (1U << 4)   /* 3x-PWM                            */
#define DRV_DRV_CTRL_OTW_REP        (1U << 2)   /* 过温警告合并到 nFAULT             */
#define DRV_DRV_CTRL_DIS_GDF        (1U << 1)   /* 禁用栅极驱动故障检测               */
#define DRV_DRV_CTRL_DIS_CPUV       (1U << 0)   /* 禁用升压泵 UVLO                   */

/* --- 0x05 Gate Drive HS ---
 *   LOCK[10:8]: 011=解锁寄存器 5/6 写入，110=锁定 (默认锁定)
 *   IDRIVEP/N_HS[7:0]: 高侧栅极拉出/拉入电流
 *   电流等级: 0=10mA,1=30,2=60,3=80,4=120,5=160,6=240,7=320,
 *              8=480,9=640,A=960,B=1280,C=1600,D=2000mA
 */
#define DRV_HS_LOCK_UNLOCK          (3U << 8)   /* LOCK=011: 允许写寄存器 5/6 */
#define DRV_HS_LOCK_LOCK            (6U << 8)   /* LOCK=110: 锁定寄存器 5/6   */
#define DRV_HS_IDRIVEP(x)           (((uint16_t)(x) & 0xFU) << 4)  /* 高侧 source 电流 */
#define DRV_HS_IDRIVEN(x)           ((uint16_t)(x) & 0xFU)          /* 高侧 sink   电流 */

/* --- 0x06 Gate Drive LS ---
 *   TDRIVE[10:8]: 峰値栅极电流驱动持续时间
 *   IDRIVEP/N_LS[7:0]: 低侧栅极拉出/拉入电流 (同上HS等级)
 */
#define DRV_LS_TDRIVE_500NS         (0U << 8)
#define DRV_LS_TDRIVE_1000NS        (1U << 8)
#define DRV_LS_TDRIVE_2000NS        (2U << 8)
#define DRV_LS_TDRIVE_4000NS        (3U << 8)
#define DRV_LS_IDRIVEP(x)           (((uint16_t)(x) & 0xFU) << 4)
#define DRV_LS_IDRIVEN(x)           ((uint16_t)(x) & 0xFU)

/* --- 0x07 OCP Control --- */
#define DRV_OCP_TRETRY_4MS          (0U << 10)  /* OCP/SEN 重试间隔 4ms   */
#define DRV_OCP_TRETRY_50US         (1U << 10)  /* OCP/SEN 重试间隔 50us  */
/* DEAD_TIME[9:8]: 驱动器内部休止时间 - 建议与HRTIM硬件死区匹配 */
#define DRV_OCP_DEAD_TIME_50NS      (0U << 8)
#define DRV_OCP_DEAD_TIME_100NS     (1U << 8)
#define DRV_OCP_DEAD_TIME_200NS     (2U << 8)   /* 与HRTIM 200ns 死区匹配  */
#define DRV_OCP_DEAD_TIME_400NS     (3U << 8)
#define DRV_OCP_MODE_LATCH          (0U << 6)   /* 锁存故障                */
#define DRV_OCP_MODE_RETRY          (1U << 6)   /* 自动重试 (推荐)       */
#define DRV_OCP_MODE_REPORT         (2U << 6)   /* 仅报告，不关断输出      */
#define DRV_OCP_MODE_DISABLED       (3U << 6)   /* 禁用 OCP               */
#define DRV_OCP_DEG_1US             (0U << 4)
#define DRV_OCP_DEG_2US             (1U << 4)   /* 2us 去抖 (推荐)        */
#define DRV_OCP_DEG_4US             (2U << 4)
#define DRV_OCP_DEG_8US             (3U << 4)
/* VDS_LVL[3:0]: VDS 过流门限电压 */
#define DRV_OCP_VDS_0V06            (0U)
#define DRV_OCP_VDS_0V13            (1U)
#define DRV_OCP_VDS_0V20            (2U)
#define DRV_OCP_VDS_0V26            (3U)
#define DRV_OCP_VDS_0V31            (4U)
#define DRV_OCP_VDS_0V45            (5U)   /* 推荐起始值 */
#define DRV_OCP_VDS_0V53            (6U)
#define DRV_OCP_VDS_0V60            (7U)
#define DRV_OCP_VDS_0V68            (8U)
#define DRV_OCP_VDS_0V75            (9U)
#define DRV_OCP_VDS_0V94            (10U)

/* --- 0x08 CSA Control --- */
#define DRV_CSA_FET                 (1U << 10)  /* 使用 Rds_on 采样（无外挂电阻）*/
#define DRV_CSA_VREF_DIV2           (1U << 9)   /* ADC满量程 = VREF/2        */
#define DRV_CSA_LS_REF              (1U << 8)   /* SHx 相对 SNx 参考           */
#define DRV_CSA_GAIN_5VV            (0U << 6)
#define DRV_CSA_GAIN_10VV           (1U << 6)
#define DRV_CSA_GAIN_20VV           (2U << 6)   /* 20V/V 推荐增益             */
#define DRV_CSA_GAIN_40VV           (3U << 6)
#define DRV_CSA_DIS_SEN             (1U << 5)   /* 禁用 SEN_OCP               */
#define DRV_CSA_CAL_A               (1U << 4)   /* A相放大器自校准             */
#define DRV_CSA_CAL_B               (1U << 3)
#define DRV_CSA_CAL_C               (1U << 2)
#define DRV_CSA_SEN_LVL_0V25        (0U)
#define DRV_CSA_SEN_LVL_0V50        (1U)        /* 0.5V SEN OCP 门限 (推荐) */
#define DRV_CSA_SEN_LVL_0V75        (2U)
#define DRV_CSA_SEN_LVL_1V00        (3U)

/* ============================================================
 * 电机状态枚举
 * ============================================================ */
typedef enum {
    MOTOR_STATE_IDLE    = 0,    /* 空闲/停止   */
    MOTOR_STATE_RUNNING = 1,    /* 正常运行    */
    MOTOR_STATE_FAULT   = 2,    /* 故障        */
} MotorState_t;

/* ============================================================
 * 电机控制结构体
 * ============================================================ */
typedef struct {
    MotorState_t    state;              /* 当前状态                 */
    float           modulationIndex;    /* 调制深度 [0.0, 1.0]      */
    float           baseFreqHz;         /* 基波频率 (Hz)             */
    uint32_t        sineTableIdx;       /* 正弦表当前索引            */
    uint32_t        sineTableSize;      /* 正弦表大小                */
    uint32_t        cmpA;               /* A相比较值                 */
    uint32_t        cmpB;               /* B相比较值                 */
    uint32_t        cmpC;               /* C相比较值                 */
} MotorCtrl_t;

/* ============================================================
 * 对外接口
 * ============================================================ */
void Motor_Init(void);
void Motor_Start(void);
void Motor_Stop(void);
void Motor_SetModulation(float index);
void Motor_SetFrequency(float freqHz);
void Motor_Update(void);            /* 主循环轮询（或在定时器中断中调用） */
void Motor_FaultHandler(void);

/* HRTIM重复中断回调（在hrtim_it.c或main中注册） */
void Motor_HRTIM_RepetitionCallback(void);

#ifdef __cplusplus
}
#endif
#endif /* __MOTOR_H */
