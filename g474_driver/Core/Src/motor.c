#include "motor.h"
#include <string.h>

/* ============================================================
 * 外部句柄（在main.c中定义）
 * ============================================================ */
extern HRTIM_HandleTypeDef hhrtim1;
extern SPI_HandleTypeDef   hspi1;
extern UART_HandleTypeDef  huart1;

/* ============================================================
 * 内部变量
 * ============================================================ */
static MotorCtrl_t  g_motor;
static uint32_t     g_sineTable[SINE_TABLE_SIZE];   /* 预计算的比较值表 */

/* ============================================================
 * SPI 帧组装宏（内部使用）
 * 帧格式: [15]R/W=0写/1读, [14:11]4bit地址, [10:0]11bit数据
 * ============================================================ */
#define DRV_SPI_WRITE(addr, data) \
    ((uint16_t)((((uint16_t)(addr) & 0x000FU) << 11U) | ((uint16_t)(data) & 0x07FFU)))
#define DRV_SPI_READ(addr) \
    ((uint16_t)(0x8000U | (((uint16_t)(addr) & 0x000FU) << 11U)))

/* ============================================================
 * 内部函数声明
 * ============================================================ */
static void     Motor_BuildSineTable(void);
static void     Motor_DRV8323_Init(void);
static void     Motor_DRV8323_WriteReg(uint8_t addr, uint16_t data);
static uint16_t Motor_DRV8323_ReadReg(uint8_t addr);
static void     Motor_SetCompare(uint32_t cmpA, uint32_t cmpB, uint32_t cmpC);
static void     Motor_EnableOutput(void);
static void     Motor_DisableOutput(void);

/* ============================================================
 * Motor_BuildSineTable
 * 预计算正弦比较值表（A相为参考，B/C通过偏移索引获取）
 *
 * 上升计数：Set=TIMPER(归零)，Reset=TIMCMP1
 *   高电平宽度 = CMP1
 *   占空比 D = CMP1 / Period = (1 + mod×sinθ) / 2
 *   → CMP1 = HalfPeriod + HalfPeriod × mod × sin(θ)
 * ============================================================ */
static void Motor_BuildSineTable(void)
{
    float half  = (float)HRTIM_PWM_HALF_PERIOD;
    float mod   = g_motor.modulationIndex;
    uint32_t n  = g_motor.sineTableSize;

    for (uint32_t i = 0; i < n; i++)
    {
        float theta  = 2.0f * (float)M_PI * (float)i / (float)n;
        float sinVal = sinf(theta);

        /* CMP1 = HalfPeriod + HalfPeriod × mod × sin(θ) */
        int32_t cmp  = (int32_t)(half + half * mod * sinVal);

        /* 限幅：CMP至少为1（避免全空层），至多为Period-1（避免全导通） */
        if (cmp < 1)                                   cmp = 1;
        if (cmp > (int32_t)(HRTIM_PWM_PERIOD - 1U))   cmp = (int32_t)(HRTIM_PWM_PERIOD - 1U);
        g_sineTable[i] = (uint32_t)cmp;
    }
}

/* ============================================================
 * Motor_DRV8323_WriteReg / ReadReg
 * SPI1: 16bit, MODE1 (CPOL=0,CPHA=1), NSS软件控制
 * ============================================================ */
static void Motor_DRV8323_WriteReg(uint8_t addr, uint16_t data)
{
    /* SPI1 配置为 16bit，1个帧 = 1个 uint16_t */
    uint16_t frame = DRV_SPI_WRITE(addr, data);
    HAL_GPIO_WritePin(DRV_SPI_NSS_PORT, DRV_SPI_NSS1_PIN, GPIO_PIN_RESET);
    HAL_SPI_Transmit(&hspi1, (uint8_t*)&frame, 1U, 10U);
    HAL_GPIO_WritePin(DRV_SPI_NSS_PORT, DRV_SPI_NSS1_PIN, GPIO_PIN_SET);
}

static uint16_t Motor_DRV8323_ReadReg(uint8_t addr)
{
    /* [15]=1读命令，[14:11]=地址，[10:0]不关心 */
    uint16_t txFrame = DRV_SPI_READ(addr);
    uint16_t rxFrame = 0U;
    HAL_GPIO_WritePin(DRV_SPI_NSS_PORT, DRV_SPI_NSS1_PIN, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, (uint8_t*)&txFrame, (uint8_t*)&rxFrame, 1U, 10U);
    HAL_GPIO_WritePin(DRV_SPI_NSS_PORT, DRV_SPI_NSS1_PIN, GPIO_PIN_SET);
    return rxFrame & 0x07FFU;  /* 只返回 [10:0] 11bit 数据 */
}

/* ============================================================
 * Motor_DRV8323_Init
 * 逐步配置 DRV8323，对应 6x-PWM 模式（HRTIM 6路独立控制）
 *
 * 注意：寄存器 0x05/0x06 默认处于锁定状态 (LOCK=110)，
 *       必须在写入时同时将 LOCK 字段设为 011 才能生效。
 * ============================================================ */
static void Motor_DRV8323_Init(void)
{
    /* 步骤 1: NSS 拉高确保空闲状态 */
    HAL_GPIO_WritePin(DRV_SPI_NSS_PORT, DRV_SPI_NSS1_PIN, GPIO_PIN_SET);

    /* 步骤 2: 拉高 ENABLE，开启 DRV8323 正常工作模式 */
    HAL_GPIO_WritePin(DRV_ENABLE_PORT, DRV_ENABLE_PIN, GPIO_PIN_SET);
    HAL_Delay(1U);   /* 等待内部升压泵 VCP 稳定，要求 ≥1ms */

    /* 步骤 3: 清除上电后可能的锁存故障
     *   CLR_FLT 是自清除位，写 1 后自动回 0
     *   同时将运行时配置一起写入 */
    Motor_DRV8323_WriteReg(DRV8323_REG_DRIVER_CTRL,
        DRV_DRV_CTRL_CLR_FLT     |   /* 清除所有锁存故障              */
        DRV_DRV_CTRL_PWM_MODE_6X |   /* 6x-PWM：6个 PWM 引脚独立控制      */
        DRV_DRV_CTRL_OTW_REP);       /* 过温警告合并到 nFAULT          */
    HAL_Delay(1U);   /* 等待 CLR_FLT 生效并自清除 */

    /* 步骤 4: 寄存器 0x04 - Driver Control 正常运行配置
     *   CLR_FLT 已自清除，此处仅保留运行配置 */
    Motor_DRV8323_WriteReg(DRV8323_REG_DRIVER_CTRL,
        DRV_DRV_CTRL_PWM_MODE_6X |   /* 6x-PWM 模式                       */
        DRV_DRV_CTRL_OTW_REP);       /* 过温警告上报 nFAULT            */

    /* 步骤 5: 寄存器 0x05 - Gate Drive HS
     *   LOCK=011 解锁寄存器 5/6 写入保护
     *   IDRIVEP_HS = 4 (120 mA 栅极 source，小 MOS 适用)
     *   IDRIVEN_HS = 6 (240 mA 栅极 sink)
     *   调整指南: t_rise ≈ Qg_total / IDRIVEP
     *              对于 Qg=20nC FET: t_rise = 20nC/120mA ≈ 167ns
     *              若 Qg 较大，提高 IDRIVEP/N 等级 */
    Motor_DRV8323_WriteReg(DRV8323_REG_GATE_DRIVE_HS,
        DRV_HS_LOCK_UNLOCK |         /* LOCK=011: 允许写入               */
        DRV_HS_IDRIVEP(0x4U) |       /* 120 mA 高侧 source 电流          */
        DRV_HS_IDRIVEN(0x6U));       /* 240 mA 高侧 sink   电流          */

    /* 步骤 6: 寄存器 0x06 - Gate Drive LS
     *   TDRIVE = 1000ns (峰値电流持续时间)
     *   IDRIVEP_LS = IDRIVEN_LS 与高侧一致 */
    Motor_DRV8323_WriteReg(DRV8323_REG_GATE_DRIVE_LS,
        DRV_LS_TDRIVE_1000NS |       /* 峰値栅极电流驱动时间 1000ns     */
        DRV_LS_IDRIVEP(0x4U) |       /* 120 mA 低侧 source 电流          */
        DRV_LS_IDRIVEN(0x6U));       /* 240 mA 低侧 sink   电流          */

    /* 步骤 7: 寄存器 0x07 - OCP Control
     *   DEAD_TIME=200ns: DRV 内部休止时间与 HRTIM 硬件死区匹配，
     *                     防止双重死区叠加导致高测软心理
     *   OCP_MODE=RETRY: 过流后自动重试，适合电机起动环境
     *   VDS_LVL=0.45V:  适合大多数 MOSFET，可根据 Rds_on 调整 */
    Motor_DRV8323_WriteReg(DRV8323_REG_OCP_CTRL,
        DRV_OCP_TRETRY_4MS     |     /* OCP 故障重试间隔 4ms            */
        DRV_OCP_DEAD_TIME_200NS|     /* 内部死区 200ns，匹配 HRTIM      */
        DRV_OCP_MODE_RETRY     |     /* VDS_OCP 自动重试                */
        DRV_OCP_DEG_2US        |     /* VDS 宿屏去抖 2us                */
        DRV_OCP_VDS_0V45);           /* VDS 过流门限 0.45V             */

    /* 步骤 8: 寄存器 0x08 - CSA Control
     *   外挂采样电阻，20V/V 增益，SEN_OCP 门限 0.5V
     *   对于 0.001Ω 电阻: 0.5V/20 = 25mA 每 1V ADC 输出对应 0.5A
     *   可根据实际采样电阻和满量程调整 CSA_GAIN */
    Motor_DRV8323_WriteReg(DRV8323_REG_CSA_CTRL,
        DRV_CSA_GAIN_20VV  |         /* 20V/V 电流放大增益              */
        DRV_CSA_SEN_LVL_0V50);       /* SEN_OCP 门限 0.5V              */

    /* 步骤 9: 读回故障寄存器，验证初始化成功 */
    HAL_Delay(1U);
    uint16_t fs1 = Motor_DRV8323_ReadReg(DRV8323_REG_FAULT_STATUS1);
    uint16_t fs2 = Motor_DRV8323_ReadReg(DRV8323_REG_FAULT_STATUS2);
    if ((fs1 != 0U) || (fs2 != 0U))
    {
        /* 仍存在故障（fs1 bit10=FAULT汇总，具体故障见 bit[9:0]） */
        g_motor.state = MOTOR_STATE_FAULT;
    }
}

/* ============================================================
 * Motor_SetCompare
 * 同步更新三相HRTIM比较值（利用预装载避免毛刺）
 * ============================================================ */
static void Motor_SetCompare(uint32_t cmpA, uint32_t cmpB, uint32_t cmpC)
{
    HRTIM_CompareCfgTypeDef cmpCfg = {0};
    cmpCfg.AutoDelayedMode    = HRTIM_AUTODELAYEDMODE_REGULAR;
    cmpCfg.AutoDelayedTimeout = 0;

    cmpCfg.CompareValue = cmpA;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_COMPAREUNIT_1, &cmpCfg);

    cmpCfg.CompareValue = cmpB;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_B, HRTIM_COMPAREUNIT_1, &cmpCfg);

    cmpCfg.CompareValue = cmpC;
    HAL_HRTIM_WaveformCompareConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_COMPAREUNIT_1, &cmpCfg);
}

/* ============================================================
 * Motor_EnableOutput / Motor_DisableOutput
 * ============================================================ */
static void Motor_EnableOutput(void)
{
    /* 启动Timer A/B/C并使能输出 */
    HAL_HRTIM_WaveformOutputStart(&hhrtim1,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

    HAL_HRTIM_WaveformCountStart_IT(&hhrtim1,
        HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_C);
}

static void Motor_DisableOutput(void)
{
    HAL_HRTIM_WaveformOutputStop(&hhrtim1,
        HRTIM_OUTPUT_TA1 | HRTIM_OUTPUT_TA2 |
        HRTIM_OUTPUT_TB1 | HRTIM_OUTPUT_TB2 |
        HRTIM_OUTPUT_TC1 | HRTIM_OUTPUT_TC2);

    HAL_HRTIM_WaveformCountStop(&hhrtim1,
        HRTIM_TIMERID_TIMER_A | HRTIM_TIMERID_TIMER_B | HRTIM_TIMERID_TIMER_C);
}

/* ============================================================
 * Motor_Init
 * ============================================================ */
void Motor_Init(void)
{
    /* 初始化控制结构体 */
    memset(&g_motor, 0, sizeof(g_motor));
    g_motor.state           = MOTOR_STATE_IDLE;
    g_motor.modulationIndex = MOTOR_MODULATION_INDEX;
    g_motor.baseFreqHz      = (float)MOTOR_SINE_FREQ_HZ;
    g_motor.sineTableSize   = SINE_TABLE_SIZE;
    g_motor.sineTableIdx    = 0;

    /* 构建正弦表 */
    Motor_BuildSineTable();

    /* 初始比较值设为半周期（零占空比偏置） */
    Motor_SetCompare(HRTIM_PWM_HALF_PERIOD,
                     HRTIM_PWM_HALF_PERIOD,
                     HRTIM_PWM_HALF_PERIOD);

    /* DRV8323初始化 */
    Motor_DRV8323_Init();
}

/* ============================================================
 * Motor_Start
 * ============================================================ */
void Motor_Start(void)
{
    if (g_motor.state == MOTOR_STATE_FAULT) return;

    /* 检查DRV8323故障引脚 */
    if (HAL_GPIO_ReadPin(DRV_NFAULT_PORT, DRV_NFAULT_PIN) == GPIO_PIN_RESET)
    {
        g_motor.state = MOTOR_STATE_FAULT;
        return;
    }

    g_motor.state        = MOTOR_STATE_RUNNING;
    g_motor.sineTableIdx = 0;

    Motor_EnableOutput();
}

/* ============================================================
 * Motor_Stop
 * ============================================================ */
void Motor_Stop(void)
{
    Motor_DisableOutput();
    /* 关闭DRV8323使能 */
    HAL_GPIO_WritePin(DRV_ENABLE_PORT, DRV_ENABLE_PIN, GPIO_PIN_RESET);
    g_motor.state = MOTOR_STATE_IDLE;
}

/* ============================================================
 * Motor_SetModulation
 * ============================================================ */
void Motor_SetModulation(float index)
{
    if (index < 0.0f) index = 0.0f;
    if (index > 1.0f) index = 1.0f;
    g_motor.modulationIndex = index;
    Motor_BuildSineTable();   /* 重建表 */
}

/* ============================================================
 * Motor_SetFrequency
 * ============================================================ */
void Motor_SetFrequency(float freqHz)
{
    if (freqHz < 1.0f)   freqHz = 1.0f;
    if (freqHz > 400.0f) freqHz = 400.0f;
    g_motor.baseFreqHz = freqHz;
    /* 重新计算正弦表大小：保持步进分辨率 */
    /* sineTableSize = PWM_FREQ / BASE_FREQ */
    uint32_t newSize = (uint32_t)((float)MOTOR_PWM_FREQ_HZ / freqHz);
    if (newSize > SINE_TABLE_SIZE) newSize = SINE_TABLE_SIZE;
    if (newSize < 4)               newSize = 4;
    g_motor.sineTableSize   = newSize;
    g_motor.sineTableIdx    = 0;
    Motor_BuildSineTable();
}

/* ============================================================
 * Motor_HRTIM_RepetitionCallback
 * 在HRTIM重复中断（每个PWM周期）中调用
 * 更新三相比较值，推进正弦波相位
 * ============================================================ */
void Motor_HRTIM_RepetitionCallback(void)
{
    if (g_motor.state != MOTOR_STATE_RUNNING) return;

    /* 检查故障引脚 */
    if (HAL_GPIO_ReadPin(DRV_NFAULT_PORT, DRV_NFAULT_PIN) == GPIO_PIN_RESET)
    {
        Motor_FaultHandler();
        return;
    }

    uint32_t n    = g_motor.sineTableSize;
    uint32_t idxA = g_motor.sineTableIdx;
    /* B相滞后120° = n/3 步，C相滞后240° = 2n/3 步 */
    uint32_t idxB = (idxA + n / 3) % n;
    uint32_t idxC = (idxA + (2 * n) / 3) % n;

    g_motor.cmpA = g_sineTable[idxA];
    g_motor.cmpB = g_sineTable[idxB];
    g_motor.cmpC = g_sineTable[idxC];

    /* 直接写入HRTIM比较寄存器（避免HAL开销） */
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_A].CMP1xR = g_motor.cmpA;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_B].CMP1xR = g_motor.cmpB;
    HRTIM1->sTimerxRegs[HRTIM_TIMERINDEX_TIMER_C].CMP1xR = g_motor.cmpC;

    /* 步进索引 */
    g_motor.sineTableIdx = (idxA + 1) % n;
}

/* ============================================================
 * Motor_FaultHandler
 * ============================================================ */
void Motor_FaultHandler(void)
{
    Motor_DisableOutput();
    HAL_GPIO_WritePin(DRV_ENABLE_PORT, DRV_ENABLE_PIN, GPIO_PIN_RESET);
    g_motor.state = MOTOR_STATE_FAULT;
}

/* ============================================================
 * Motor_Update
 * 主循环轮询：监控故障状态（低频）
 * ============================================================ */
void Motor_Update(void)
{
    if (g_motor.state == MOTOR_STATE_RUNNING)
    {
        if (HAL_GPIO_ReadPin(DRV_NFAULT_PORT, DRV_NFAULT_PIN) == GPIO_PIN_RESET)
        {
            Motor_FaultHandler();
        }
    }
}

/* ============================================================
 * HAL HRTIM回调钩子
 * 在stm32g4xx_hal_hrtim.c的弱函数基础上重写
 * ============================================================ */
void HAL_HRTIM_RepetitionEventCallback(HRTIM_HandleTypeDef *hhrtim,
                                        uint32_t TimerIdx)
{
    if (TimerIdx == HRTIM_TIMERINDEX_TIMER_A)
    {
        Motor_HRTIM_RepetitionCallback();
    }
}
