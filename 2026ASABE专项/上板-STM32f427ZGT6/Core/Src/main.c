/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Car__.h"
#include "my_uart.h"
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
#define RX_BUF_SIZE 16

#define PAL_UART_BAUDRATE 115200U
#define PAL_RX_FRAME_LEN 12U
#define PAL_TX_FRAME_LEN 10U
#define PAL_TX_FRAME_HEAD 0x55U
#define PAL_PACKET_TAIL 0xAAU
#define PAL_LEFT_NEW_FRAME_FLAG 0x01U
#define PAL_RIGHT_NEW_FRAME_FLAG 0x02U
#define DUINO_FRAME_LEN 6U
#define MV_FRAME_LEN 6U
#define UART_FRAME_QUEUE_DEPTH 8U
#define UART_FRAME_MAX_LEN RX_BUF_SIZE
// 提前伸出一点，确保能够用车速除草
#define PAL_LEFT_OFFSET_BIAS -30
#define PAL_RIGHT_OFFSET_BIAS -30
#define MV_LEFT_OFFSET_Y_BIAS 0
#define MV_RIGHT_OFFSET_Y_BIAS 0
#define RTOS_TICKS_PER_MS 2U
#define MOTION_TASK_PERIOD_TICKS 1U
#define TELEMETRY_TASK_PERIOD_TICKS (50U * RTOS_TICKS_PER_MS)


typedef enum {
    UART_FRAME_BT = 0,
    UART_FRAME_PAL,
    UART_FRAME_DUINO,
    UART_FRAME_MV_R,
    UART_FRAME_MV_L
} UartFrameSource_t;

typedef struct {
    UartFrameSource_t source;
    uint8_t len;
    uint8_t data[UART_FRAME_MAX_LEN];
} UartFrameMsg_t;


uint8_t uart_rx_buf[RX_BUF_SIZE];
uint8_t uart_rx_buf_pal[RX_BUF_SIZE];
uint8_t uart_rx_buf_duino[RX_BUF_SIZE];
uint8_t uart_rx_buf_mv_l[RX_BUF_SIZE];
uint8_t uart_rx_buf_mv_r[RX_BUF_SIZE];


static uint8_t uart7_tx_buf[PAL_TX_FRAME_LEN];
static volatile uint8_t uart7_tx_busy = 0U;
static volatile uint8_t pi_stats_tx_pending = 0U;
static uint8_t pi_stats_tx_seq = 0U;
static osMessageQueueId_t uart_frame_queue = NULL;
volatile uint8_t pal_rx_break_enable = 1U;
volatile uint8_t pal_rx_break_hit = 0U;
volatile uint8_t pal_rx_last_byte = 0U;
volatile uint32_t pal_rx_byte_count = 0U;

volatile uint16_t rx_head = 0;
volatile uint16_t rx_head_pal = 0;
volatile uint16_t rx_head_duino = 0;
volatile uint16_t rx_head_mv_l = 0;
volatile uint16_t rx_head_mv_r = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
volatile uint8_t reset_mcu = 0;
float Kp = 0.0f, Ki = 0.0f, Kd = 0.0f;

typedef enum {
    WORK_OFF = 0,   // enable=0
    WORK_ARMING,    // enable rises, wait before active
    WORK_ACTIVE     // active work phase
} WorkPhase_t;

static WorkPhase_t g_work_phase = WORK_OFF;
static uint8_t g_prev_enable = 0U;
static uint32_t g_work_deadline = 0U;       // TCM tick
#define WORK_START_DELAY_TICKS 20000U      // 100tick = 1ms, 100000tick = 1s



static inline void Main_System_EnterWork(uint32_t now_ticks)
{
    g_work_phase = WORK_ARMING;
    g_work_deadline = now_ticks + WORK_START_DELAY_TICKS;

    MotorPidController_Reset(&qExtend);
    MotorPidController_Reset(&qRotate);
    MotorPidController_Reset(&qOpenmv);

    set_waiting(&qExtend);
    set_waiting(&qRotate);
    set_waiting(&qOpenmv);

    WindowJudge_Reset(&wj_l);
    WindowJudge_Reset(&wj_r);

}

static inline void Main_System_ExitWork(void)
{
    g_work_phase = WORK_OFF;

    set_orign(&qExtend);
    set_orign(&qRotate);
    set_orign(&qOpenmv);

    WindowJudge_Reset(&wj_l);
    WindowJudge_Reset(&wj_r);

}

static inline void Main_System_UpdateWorkPhase(void)
{
    uint8_t en = ArduinoWorkCmd.is_enable;
    uint32_t now_ticks = (uint32_t)TCM_Readtimer6count();

    if (en == 1U && g_prev_enable == 0U) {
        Main_System_EnterWork(now_ticks);
    } else if (en == 0U && g_prev_enable == 1U) {
        Main_System_ExitWork();
    }
    g_prev_enable = en;

    if (g_work_phase == WORK_ARMING) {
        if ((int32_t)(now_ticks - g_work_deadline) >= 0) {
            g_work_phase = WORK_ACTIVE;
            WindowJudge_EnterWork(&wj_l, MVpacket_l.in_window);
            WindowJudge_EnterWork(&wj_r, MVpacket_r.in_window);

        }
    }
}

void StartReceive(UART_HandleTypeDef *huart);
static int16_t read_i16_le(const uint8_t *p);
static float read_f32_le(const uint8_t *p);
static void PiStats_RequestSend(void);
static void PiStats_ServiceTx(void);
static void PiStats_Pack(uint8_t *buf);
static void Main_RunNormalLogic(void);
static void Main_RunPlantClassifyTask(void);
static void Main_RunSowTask(void);
static void Main_RunPlantActuationTask(uint8_t left);
static void Main_RunPlantActuationTestTask(void);
static void Main_RunPlantActuationTask2(void);
static void Main_DoPlantSow(void);
static void Main_DoPlantSowBoth(void);
static void Main_DoType0PlantSowSeedBoth(void);
static void Comm_StartReceiveAll(void);
static void UartFrame_PostFromIsr(UartFrameSource_t source, const uint8_t *data, uint8_t len);
static void UartFrame_Parse(const UartFrameMsg_t *msg);

#define PLANT_CLASSIFY_TASK 0U
#define PLANT_ACTUATION_TASK 0U
#define PLANT_ACTUATION_TASK2 0U
#define MAIN_TASK 1U
#define PLANT_SOW_TRIGGER_TEST 0U
#define PLANT_SOW_TRIGGER_TEST_BOTH 0U
#define PLANT_SOW_CLASSIFY_TEST 0U
#define SOW_SEQUENCE_TEST 1U
#define TRACK_ROTATE_TEST 0U
#define ENCODER_READ_TEST 0U

#define PLANT_SOW_TRIGGER_TEST_LEFT 1U
#define PLANT_ACTUATION_TASK2_LEFT_WEED_RIGHT_SOW 1U
#define PLANT_SOW_MOVE_DOWN_FAST_TICKS 97500U
#define PLANT_SOW_MOVE_DOWN_SLOW_TICKS 102500U
#define PLANT_SOW_MOVE_UP_BRAKE_TICKS 5000U
#define PLANT_SOW_MOVE_UP_FAST_TICKS 103500U
#define PLANT_SOW_MOVE_UP_SLOW_TICKS 115500U
#define PLANT_SOW_TRIGGER_DX_MIN (-100)
#define PLANT_SOW_TRIGGER_DX_MAX (0)
#define PLANT_SOW_TRIGGER_HOLD_TICKS 5000U
#define TYPE0_SOW_MAX_PER_PLANT 2U
#define PLANT_SOW_SEED_FWD_FAST_TICKS 4000U
#define PLANT_SOW_SEED_FWD_SLOW_TICKS 13000U
#define PLANT_SOW_SEED_HOLD_TICKS 43000U
#define PLANT_SOW_SEED_REV_FAST_TICKS 47000U
#define PLANT_SOW_SEED_REV_SLOW_TICKS 56000U
int8_t step = 0;
volatile int32_t encoder_pRotate_test = 0;
volatile int32_t encoder_pExtend_test = 0;
volatile int32_t encoder_qRotate_test = 0;
volatile int32_t encoder_qExtend_test = 0;
volatile int32_t ccr = 0;
volatile uint8_t track_state_test = 0;
volatile uint8_t track_target_in_limit_test = 0;
volatile int32_t track_encoder_togo_test = 0;
volatile uint8_t track_can_extend_test = 0;
volatile uint8_t track_is_unstable_test = 0;
volatile int16_t track_rotate_pwm_test = 0;
volatile int16_t track_extend_pwm_test = 0;
volatile float track_kp_term_test = 0.0f;
volatile float track_ki_term_test = 0.0f;
volatile float track_kd_term_test = 0.0f;
volatile float pal_track_target_filter_alpha = 0.08f;
volatile float pal_track_speed_filter_alpha = 0.005f;
volatile float openmv_offset_filter_alpha = 0.08f;
volatile uint32_t plant_actuation_cycle_test = 0;
volatile uint8_t plant_actuation_left_test = 0;
volatile uint8_t plant_actuation_enable_test = 0;
volatile uint8_t plant_sow_move_state_test = 0;
volatile int16_t plant_sow_seed_dx_test = 0;

typedef enum {
    PLANT_SOW_MOVE_IDLE = 0,
    PLANT_SOW_MOVE_DOWN,
    PLANT_SOW_MOVE_UP
} PlantSowMoveState_t;

typedef struct {
    PlantSowMoveState_t state;
    uint32_t start_time;
    Soft_Motor *move_m;
} PlantSowMoveCtx_t;

static PlantSowMoveCtx_t plant_sow_move_ctx = {
    PLANT_SOW_MOVE_IDLE, 0U, NULL
};

static PlantSowMoveCtx_t plant_sow_seed_ctx = {
    PLANT_SOW_MOVE_IDLE, 0U, NULL
};
/**
  * @brief 运行植株分类任务。
  * @note  首次调用时根据左右 OpenMV 的 in_window 状态进入窗口判定流程；
  *        后续调用更新分类器并同步 LED 指示状态；
  *        自动触发0->1的使能信号时，进入窗口判定流程。 
  * @retval None
  */
static void Main_RunPlantClassifyTask(void)
{
    // inwindows:1->000000->1 
    PlantClassifier_Update();
    // LED update by plant type
    LED_UpdateByType();
}

static void Main_RunSowTask(void)
{
    //bug:delay too long
    Execute_Sow_Sequence(&wj_l, &MVpacket_l, &qSowMove, &qSowSeed);
    Execute_Sow_Sequence(&wj_r, &MVpacket_r, &pSowMove, &pSowSeed);
}

static Soft_Motor *Main_GetPlantSowMoveMotor(uint8_t left)
{
    // left=1: left weeding + right sowing; left=0: left sowing + right weeding.
    return (left == 1U) ? &pSowMove : &qSowMove;
}

static Soft_Motor *Main_GetPlantSowSeedMotor(uint8_t left)
{
    // left=1: left weeding + right sowing; left=0: left sowing + right weeding.
    return (left == 1U) ? &pSowSeed : &qSowSeed;
}

static int16_t Main_GetPlantSowOffsetX(uint8_t left)
{
    int16_t offset_x = 0;

    __disable_irq();
    offset_x = (left == 1U) ? MVpacket_r.offset_x : MVpacket_l.offset_x;
    __enable_irq();

    return offset_x;
}

static uint8_t Main_GetPlantSowStable(uint8_t left)
{
    uint8_t stable = 0U;

    __disable_irq();
    stable = (left == 1U) ? MVpacket_r.is_plant_find : MVpacket_l.is_plant_find;
    __enable_irq();

    return stable;
}

static void Main_StartPlantSowMove(Soft_Motor *move_m, PlantSowMoveState_t state)
{
    if (move_m == NULL) {
        return;
    }
    plant_sow_move_ctx.move_m = move_m;
    plant_sow_move_ctx.state = state;
    plant_sow_move_ctx.start_time = (uint32_t)TCM_Readtimer6count();
    plant_sow_move_state_test = (uint8_t)state;
}

static void Main_UpdatePlantSowMove(void)
{
    Soft_Motor *move_m = plant_sow_move_ctx.move_m;
    uint32_t now = (uint32_t)TCM_Readtimer6count();
    uint32_t dt = now - plant_sow_move_ctx.start_time;

    if (move_m == NULL) {
        plant_sow_move_ctx.state = PLANT_SOW_MOVE_IDLE;
        plant_sow_move_state_test = PLANT_SOW_MOVE_IDLE;
        return;
    }

    switch (plant_sow_move_ctx.state) {
        case PLANT_SOW_MOVE_DOWN:
            if (dt < PLANT_SOW_MOVE_DOWN_FAST_TICKS) {
                Soft_Motor_set_pwm(move_m, 5);
            } else if (dt < PLANT_SOW_MOVE_DOWN_SLOW_TICKS) {
                Soft_Motor_set_pwm(move_m, 1);
            } else {
                Soft_Motor_brake(move_m);
                plant_sow_move_ctx.state = PLANT_SOW_MOVE_IDLE;
                plant_sow_move_state_test = PLANT_SOW_MOVE_IDLE;
            }
            break;

        case PLANT_SOW_MOVE_UP:
            if (dt <= PLANT_SOW_MOVE_UP_BRAKE_TICKS) {
                Soft_Motor_brake(move_m);
            } else if (dt <= PLANT_SOW_MOVE_UP_FAST_TICKS) {
                Soft_Motor_set_pwm(move_m, -5);
            } else if (dt <= PLANT_SOW_MOVE_UP_SLOW_TICKS) {
                Soft_Motor_set_pwm(move_m, -1);
            } else {
                Soft_Motor_brake(move_m);
                plant_sow_move_ctx.state = PLANT_SOW_MOVE_IDLE;
                plant_sow_move_state_test = PLANT_SOW_MOVE_IDLE;
            }
            break;

        default:
            Soft_Motor_brake(move_m);
            plant_sow_move_ctx.state = PLANT_SOW_MOVE_IDLE;
            plant_sow_move_state_test = PLANT_SOW_MOVE_IDLE;
            break;
    }
}

static void Main_StartPlantSowSeed(Soft_Motor *seed_m)
{
    //正在投种则退出新投种任务的分配
    if (seed_m == NULL || plant_sow_seed_ctx.state != PLANT_SOW_MOVE_IDLE) {
        return;
    }

    plant_sow_seed_ctx.move_m = seed_m;
    //设置当前投种状态为开始运行 | 记录开始时间
    plant_sow_seed_ctx.state = PLANT_SOW_MOVE_DOWN;
    plant_sow_seed_ctx.start_time = (uint32_t)TCM_Readtimer6count();
}

static void Main_UpdatePlantSowSeed(void)
{
    Soft_Motor *seed_m = plant_sow_seed_ctx.move_m;
    uint32_t now = (uint32_t)TCM_Readtimer6count();
    uint32_t dt = now - plant_sow_seed_ctx.start_time;
    uint32_t rev_slow_ticks = PLANT_SOW_SEED_REV_SLOW_TICKS;
    uint32_t fwd_slow_ticks = PLANT_SOW_SEED_FWD_SLOW_TICKS;

    if (seed_m == &qSowSeed) {
        rev_slow_ticks += 1000U;
        fwd_slow_ticks += 2000U;
    }

    if (seed_m == NULL) {
        plant_sow_seed_ctx.state = PLANT_SOW_MOVE_IDLE;
        return;
    }
    //待机状态 brake 刹车 | return
    if (plant_sow_seed_ctx.state != PLANT_SOW_MOVE_DOWN) {
        Soft_Motor_brake(seed_m);
        plant_sow_seed_ctx.state = PLANT_SOW_MOVE_IDLE;
        return;
    }

    if (dt <= PLANT_SOW_SEED_FWD_FAST_TICKS) {
        Soft_Motor_set_pwm(seed_m, 9);
    } else if (dt <= fwd_slow_ticks) {
        Soft_Motor_set_pwm(seed_m, 2);
    } else if (dt <= PLANT_SOW_SEED_HOLD_TICKS) {
        Soft_Motor_brake(seed_m);
    } else if (dt <= PLANT_SOW_SEED_REV_FAST_TICKS) {
        Soft_Motor_set_pwm(seed_m, -9);
    } else if (dt <= rev_slow_ticks) {
        Soft_Motor_set_pwm(seed_m, -2);
    } else {
        Soft_Motor_brake(seed_m);
        plant_sow_seed_ctx.state = PLANT_SOW_MOVE_IDLE;
    }
}

static uint8_t Main_StartPlantSowSeedCtx(PlantSowMoveCtx_t *ctx, Soft_Motor *seed_m)
{
    if (ctx == NULL || seed_m == NULL || ctx->state != PLANT_SOW_MOVE_IDLE) {
        return 0U;
    }

    ctx->move_m = seed_m;
    ctx->state = PLANT_SOW_MOVE_DOWN;
    ctx->start_time = (uint32_t)TCM_Readtimer6count();
    return 1U;
}

static void Main_UpdatePlantSowSeedCtx(PlantSowMoveCtx_t *ctx)
{
    Soft_Motor *seed_m;
    uint32_t now;
    uint32_t dt;
    uint32_t rev_slow_ticks = PLANT_SOW_SEED_REV_SLOW_TICKS;
    uint32_t fwd_slow_ticks = PLANT_SOW_SEED_FWD_SLOW_TICKS;

    if (ctx == NULL) {
        return;
    }

    seed_m = ctx->move_m;
    now = (uint32_t)TCM_Readtimer6count();
    dt = now - ctx->start_time;

    if (seed_m == &qSowSeed) {
        rev_slow_ticks += 1000U;
        fwd_slow_ticks += 2000U;
    }

    if (seed_m == NULL) {
        ctx->state = PLANT_SOW_MOVE_IDLE;
        return;
    }

    if (ctx->state != PLANT_SOW_MOVE_DOWN) {
        Soft_Motor_brake(seed_m);
        ctx->state = PLANT_SOW_MOVE_IDLE;
        return;
    }

    if (dt <= PLANT_SOW_SEED_FWD_FAST_TICKS) {
        Soft_Motor_set_pwm(seed_m, 9);
    } else if (dt <= fwd_slow_ticks) {
        Soft_Motor_set_pwm(seed_m, 2);
    } else if (dt <= PLANT_SOW_SEED_HOLD_TICKS) {
        Soft_Motor_brake(seed_m);
    } else if (dt <= PLANT_SOW_SEED_REV_FAST_TICKS) {
        Soft_Motor_set_pwm(seed_m, -9);
    } else if (dt <= rev_slow_ticks) {
        Soft_Motor_set_pwm(seed_m, -2);
    } else {
        Soft_Motor_brake(seed_m);
        ctx->state = PLANT_SOW_MOVE_IDLE;
    }
}

static void Main_DoPlantSow(void)
{   //锁（分配一次任务）
    static uint8_t trigger_armed = 1U;
    static uint8_t trigger_timing = 0U;
    static uint32_t trigger_start_time = 0U;
    uint8_t left = plant_actuation_left_test;
    int16_t offset_x = Main_GetPlantSowOffsetX(left);
    uint8_t stable = Main_GetPlantSowStable(left);
    uint32_t now = (uint32_t)TCM_Readtimer6count();
    uint8_t in_trigger_window =
        (stable == 1U &&
         offset_x >= PLANT_SOW_TRIGGER_DX_MIN &&
         offset_x <= PLANT_SOW_TRIGGER_DX_MAX);

    plant_sow_seed_dx_test = offset_x;

    if (ArduinoWorkCmd.is_enable == 1U) {
        if (in_trigger_window && trigger_armed) {
            if (!trigger_timing) {
                trigger_timing = 1U;
                trigger_start_time = now;
            } else if ((uint32_t)(now - trigger_start_time) >= PLANT_SOW_TRIGGER_HOLD_TICKS) {
                Main_StartPlantSowSeed(Main_GetPlantSowSeedMotor(left));
                trigger_armed = 0U;
                trigger_timing = 0U;
            }
        } else {
            trigger_timing = 0U;
            if (!in_trigger_window) {
                trigger_armed = 1U;
            }
        }
    } else {
        trigger_timing = 0U;
        trigger_armed = 1U;
    }
    Main_UpdatePlantSowSeed();
}

static void Main_DoPlantSowBoth(void)
{
    typedef struct {
        uint8_t trigger_armed;
        uint8_t trigger_timing;
        uint32_t trigger_start_time;
        PlantSowMoveCtx_t seed_ctx;
    } PlantSowBothSide_t;

    static PlantSowBothSide_t left_side = {
        1U, 0U, 0U, { PLANT_SOW_MOVE_IDLE, 0U, NULL }
    };
    static PlantSowBothSide_t right_side = {
        1U, 0U, 0U, { PLANT_SOW_MOVE_IDLE, 0U, NULL }
    };
    uint32_t now = (uint32_t)TCM_Readtimer6count();
    int16_t offset_x_l;
    int16_t offset_x_r;
    uint8_t stable_l;
    uint8_t stable_r;
    uint8_t in_trigger_window_l;
    uint8_t in_trigger_window_r;

    __disable_irq();
    offset_x_l = MVpacket_l.offset_x;
    offset_x_r = MVpacket_r.offset_x;
    stable_l = MVpacket_l.is_plant_find;
    stable_r = MVpacket_r.is_plant_find;
    __enable_irq();

    in_trigger_window_l =
        (stable_l == 1U &&
         offset_x_l >= PLANT_SOW_TRIGGER_DX_MIN &&
         offset_x_l <= PLANT_SOW_TRIGGER_DX_MAX);
    in_trigger_window_r =
        (stable_r == 1U &&
         offset_x_r >= PLANT_SOW_TRIGGER_DX_MIN &&
         offset_x_r <= PLANT_SOW_TRIGGER_DX_MAX);

    if (ArduinoWorkCmd.is_enable == 1U) {
        // left side sowing
        if (in_trigger_window_l && left_side.trigger_armed) {
            if (!left_side.trigger_timing) {
                left_side.trigger_timing = 1U;
                left_side.trigger_start_time = now;
            } else if ((uint32_t)(now - left_side.trigger_start_time) >= PLANT_SOW_TRIGGER_HOLD_TICKS) {
                Main_StartPlantSowSeedCtx(&left_side.seed_ctx, &qSowSeed);
                left_side.trigger_armed = 0U;
                left_side.trigger_timing = 0U;
            }
        } else {
            left_side.trigger_timing = 0U;
            if (!in_trigger_window_l) {
                left_side.trigger_armed = 1U;
            }
        }
        // right side sowing
        if (in_trigger_window_r && right_side.trigger_armed) {
            if (!right_side.trigger_timing) {
                right_side.trigger_timing = 1U;
                right_side.trigger_start_time = now;
            } else if ((uint32_t)(now - right_side.trigger_start_time) >= PLANT_SOW_TRIGGER_HOLD_TICKS) {
                Main_StartPlantSowSeedCtx(&right_side.seed_ctx, &pSowSeed);
                right_side.trigger_armed = 0U;
                right_side.trigger_timing = 0U;
            }
        } else {
            right_side.trigger_timing = 0U;
            if (!in_trigger_window_r) {
                right_side.trigger_armed = 1U;
            }
        }
    } else {
        left_side.trigger_timing = 0U;
        left_side.trigger_armed = 1U;
        right_side.trigger_timing = 0U;
        right_side.trigger_armed = 1U;
    }

    Main_UpdatePlantSowSeedCtx(&left_side.seed_ctx);
    Main_UpdatePlantSowSeedCtx(&right_side.seed_ctx);
}

typedef struct {
    uint8_t trigger_armed;
    uint8_t trigger_timing;
    uint8_t type0_seen;
    uint8_t sow_count;
    uint32_t trigger_start_time;
    PlantSowMoveCtx_t seed_ctx;
} PlantType0SowSeedSide_t;

static void Main_DoType0PlantSowSeedSide(PlantType0SowSeedSide_t *side,
                                         const WindowJudge_t *wj,
                                         const MVPacket_t *packet,
                                         Soft_Motor *seed_m)
{
    uint32_t now;
    uint8_t is_type0;
    uint8_t stable;
    int16_t offset_x;
    uint8_t in_trigger_window;
    uint8_t sow_limit_reached;

    if (side == NULL || wj == NULL || packet == NULL || seed_m == NULL) {
        return;
    }
    now = (uint32_t)TCM_Readtimer6count();
    __disable_irq();
    is_type0 = (wj->type_now == 0U) ? 1U : 0U;
    stable = packet->is_plant_find;
    offset_x = packet->offset_x;
    __enable_irq();

    if (ArduinoWorkCmd.is_enable != 1U || !is_type0) {
        side->trigger_timing = 0U;
        side->trigger_armed = 1U;
        side->type0_seen = 0U;
        side->sow_count = 0U;
        Main_UpdatePlantSowSeedCtx(&side->seed_ctx);
        return;
    }
    // Start a new type0 plant count.
    if (!side->type0_seen) {
        side->type0_seen = 1U;
        side->sow_count = 0U;
        side->trigger_armed = 1U;
        side->trigger_timing = 0U;
    }
    in_trigger_window =
        (stable == 1U &&
         offset_x >= PLANT_SOW_TRIGGER_DX_MIN &&
         offset_x <= PLANT_SOW_TRIGGER_DX_MAX);
    sow_limit_reached =
        (TYPE0_SOW_MAX_PER_PLANT != 0U &&
         side->sow_count >= TYPE0_SOW_MAX_PER_PLANT);

    if (sow_limit_reached) {
        side->trigger_timing = 0U;
        side->trigger_armed = 0U;
        Main_UpdatePlantSowSeedCtx(&side->seed_ctx);
        return;
    }

    if (in_trigger_window && side->trigger_armed) {
        if (!side->trigger_timing) {
            side->trigger_timing = 1U;
            side->trigger_start_time = now;
        } else if ((uint32_t)(now - side->trigger_start_time) >= PLANT_SOW_TRIGGER_HOLD_TICKS) {
            if (Main_StartPlantSowSeedCtx(&side->seed_ctx, seed_m)) {
                side->sow_count++;
                if (TYPE0_SOW_MAX_PER_PLANT != 0U &&
                    side->sow_count >= TYPE0_SOW_MAX_PER_PLANT) {
                    side->trigger_armed = 0U;
                }
            }
            side->trigger_timing = 0U;
        }
    } else {
        side->trigger_timing = 0U;
        if (!in_trigger_window) {
            side->trigger_armed = 1U;
        }
    }

    Main_UpdatePlantSowSeedCtx(&side->seed_ctx);
}

static void Main_DoType0PlantSowSeedBoth(void)
{
    static PlantType0SowSeedSide_t left_side = {
        1U, 0U, 0U, 0U, 0U, { PLANT_SOW_MOVE_IDLE, 0U, NULL }
    };
    static PlantType0SowSeedSide_t right_side = {
        1U, 0U, 0U, 0U, 0U, { PLANT_SOW_MOVE_IDLE, 0U, NULL }
    };

    Main_DoType0PlantSowSeedSide(&left_side, &wj_l, &MVpacket_l, &qSowSeed);
    Main_DoType0PlantSowSeedSide(&right_side, &wj_r, &MVpacket_r, &pSowSeed);
}

static void Main_RunPlantActuationTask(uint8_t left)
{
    // left=1: left weeding + right sowing; left=0: left sowing + right weeding.
    if (left == 1U) {
        openmv_extend_left(200, 5);
        openmv_extend_right(200, 5);
        set_orign(&pRotate);
        set_orign(&pExtend);
        track_and_extend_left(1000, 2000, 80, 2000, 50000);
    } else {
        openmv_extend_left(200, 5);
        openmv_extend_right(200, 5);
        set_orign(&qRotate);
        set_orign(&qExtend);
        track_and_extend_right(1000, 2000, 80, 2000, 50000);
    }
}

static void Main_RunPlantActuationTestTask(void)
{
    static uint8_t prev_enable = 0U;
    static uint32_t cycle_count = 0U;
    static uint8_t current_left = 0U;  // 0 表示左边为投种
    uint8_t enable = ArduinoWorkCmd.is_enable;

    plant_actuation_enable_test = enable;

    if (enable == 1U && prev_enable == 0U) {
        cycle_count++;
        // first time enable, current_left = 1 , right sowing
        //current_left = 0 ;  // 0 left sowing, 1 right sowing
        current_left = 1U;
        plant_actuation_cycle_test = cycle_count;
        plant_actuation_left_test = current_left;
        // 0->1 下降，1->0 上升
        // SowMove 电机
        Main_StartPlantSowMove(Main_GetPlantSowMoveMotor(current_left),
                               PLANT_SOW_MOVE_DOWN);
    } else if (enable == 0U && prev_enable == 1U) {
        Main_StartPlantSowMove(Main_GetPlantSowMoveMotor(current_left),
                               PLANT_SOW_MOVE_UP);
    }

    prev_enable = enable;

    if (enable == 1U) {
        Main_RunPlantActuationTask(current_left);
    } else {
        set_orign(&qRotate);
        set_orign(&qExtend);
        set_orign(&pRotate);
        set_orign(&pExtend);
        set_waiting(&qOpenmv);
        set_waiting(&pOpenmv);
    }

    Main_UpdatePlantSowMove();
    Main_DoPlantSow();
}

static void Main_RunPlantActuationTask2(void)
{
    static uint8_t prev_enable = 0U;
    static uint32_t cycle_count = 0U;
    static uint8_t sow_move_lowered = 0U;
    const uint8_t current_left = PLANT_ACTUATION_TASK2_LEFT_WEED_RIGHT_SOW ? 1U : 0U;
    uint8_t enable = ArduinoWorkCmd.is_enable;

    plant_actuation_enable_test = enable;
    plant_actuation_left_test = current_left;

    if (enable == 1U && prev_enable == 0U && !sow_move_lowered) {
        cycle_count++;
        plant_actuation_cycle_test = cycle_count;
        sow_move_lowered = 1U;
        Main_StartPlantSowMove(Main_GetPlantSowMoveMotor(current_left),
                               PLANT_SOW_MOVE_DOWN);
    }

    prev_enable = enable;

    if (enable == 1U) {
        Main_RunPlantActuationTask(current_left);
    } else {
        set_orign(&qRotate);
        set_orign(&qExtend);
        set_orign(&pRotate);
        set_orign(&pExtend);
        set_waiting(&qOpenmv);
        set_waiting(&pOpenmv);
    }

    Main_UpdatePlantSowMove();
    Main_DoPlantSow();
}

static void Main_RunNormalLogic(void)
{
#if PLANT_CLASSIFY_TASK
    static uint8_t classify_inited = 0U;

    step = 30;
    if (!classify_inited) {
        WindowJudge_EnterWork(&wj_l, MVpacket_l.in_window);
        WindowJudge_EnterWork(&wj_r, MVpacket_r.in_window);
        classify_inited = 1U;
    }
    Main_RunPlantClassifyTask();
    return;
#elif PLANT_ACTUATION_TASK
    step = 31;
    Main_RunPlantActuationTestTask();
    //Soft_Motor_set_pwm(&pSowSeed, 1);      // 正转  逆时针 （挡住上）
    return;
#elif PLANT_ACTUATION_TASK2
    step = 36;
    Main_RunPlantActuationTask2();
    return;
#elif MAIN_TASK
    // change work phase, reset step and motors, openmv enterwork set: 0->1 
    Main_System_UpdateWorkPhase();
    if (g_work_phase == WORK_OFF) {
        step = 0;
        set_orign(&qExtend);
        set_orign(&qRotate);
        set_waiting(&qOpenmv);
        set_orign(&pExtend);
        set_orign(&pRotate);
        set_waiting(&pOpenmv);
    } else if (g_work_phase == WORK_ARMING) {
        set_waiting(&qExtend);
        set_waiting(&qRotate);
        set_waiting(&qOpenmv);
        set_waiting(&pExtend);
        set_waiting(&pRotate);  
        set_waiting(&pOpenmv);
    } else {
        step = 1;
        Main_RunPlantClassifyTask();  // classify 
        track_and_extend_left(1000, 2000, 80, 2500, 70000);
        openmv_extend_left(300, 5);
        track_and_extend_right(1000, 2000, 80, 2500, 70000);
        openmv_extend_right(300, 5);
    }
    Main_DoType0PlantSowSeedBoth();  // sow
    //Main_RunSowTask();
#elif PLANT_SOW_TRIGGER_TEST
    step = 33;
    ArduinoWorkCmd.is_enable = 1U;
    plant_actuation_left_test = 0;  // 0 : left sowing, 1: right sowing
    plant_actuation_enable_test = ArduinoWorkCmd.is_enable;
    openmv_extend_left(600, 5);
    //openmv_extend_right(600, 5);
    Main_DoPlantSow();
    return;
#elif PLANT_SOW_TRIGGER_TEST_BOTH
    step = 34;
    ArduinoWorkCmd.is_enable = 1U;  // both side sowing
    plant_actuation_enable_test = ArduinoWorkCmd.is_enable;
    Main_DoPlantSowBoth();
    return;
#elif PLANT_SOW_CLASSIFY_TEST
    step = 35;
    static uint8_t classifier_inited = 0U;

    ArduinoWorkCmd.is_enable = 1U;

    if (!classifier_inited) {
        WindowJudge_EnterWork(&wj_l, MVpacket_l.in_window);
        WindowJudge_EnterWork(&wj_r, MVpacket_r.in_window);
        classifier_inited = 1U;
    }

    Main_RunPlantClassifyTask();
    Main_DoType0PlantSowSeedBoth();
    return;
#elif SOW_SEQUENCE_TEST             // f2 : down -> sowseed -> up
    step = 32;
    static uint8_t sow_sequence_test_step = 0U;
    static uint32_t sow_sequence_test_time = 0U;
    uint32_t sow_sequence_now = (uint32_t)TCM_Readtimer6count();
    uint32_t sow_sequence_dt = sow_sequence_now - sow_sequence_test_time;

    switch (sow_sequence_test_step) {
        case 0U:
            sow_sequence_test_time = sow_sequence_now;
            sow_sequence_test_step = 1U;
            break;

        case 1U:
            Soft_Motor_brake(&pSowMove);
            Soft_Motor_brake(&pSowSeed);
            if (sow_sequence_dt >= 100000U) {
                sow_sequence_test_time = sow_sequence_now;
                sow_sequence_test_step = 2U;
            }
            break;

        case 2U:
            if (sow_sequence_dt < 100000U) {
                Soft_Motor_set_pwm(&pSowMove, 5);
            } else if (sow_sequence_dt < 107500U) {
                Soft_Motor_set_pwm(&pSowMove, 1);
            } else {
                Soft_Motor_brake(&pSowMove);
                sow_sequence_test_time = sow_sequence_now;
                sow_sequence_test_step = 3U;
            }
            break;

        case 3U:
            if (sow_sequence_dt <= 3000U) {
                Soft_Motor_set_pwm(&pSowSeed, 9);
            } else if (sow_sequence_dt <= 11000U) {
                Soft_Motor_set_pwm(&pSowSeed, 2);
            } else if (sow_sequence_dt <= 41000U) {
                Soft_Motor_brake(&pSowSeed);
            } else if (sow_sequence_dt <= 44000U) {
                Soft_Motor_set_pwm(&pSowSeed, -9);
            } else if (sow_sequence_dt <= 52000U) {
                Soft_Motor_set_pwm(&pSowSeed, -2);
            } else {
                Soft_Motor_brake(&pSowSeed);
                sow_sequence_test_time = sow_sequence_now;
                sow_sequence_test_step = 4U;
            }
            break;

        case 4U:
            if (sow_sequence_dt <= 5000U) {
                Soft_Motor_brake(&pSowMove);
            } else if (sow_sequence_dt <= 106000U) {
                Soft_Motor_set_pwm(&pSowMove, -5);
            } else if (sow_sequence_dt <= 118500U) {
                Soft_Motor_set_pwm(&pSowMove, -1);
            } else {
                Soft_Motor_brake(&pSowMove);
                sow_sequence_test_step = 5U;
            }
            break;

        default:
            Soft_Motor_brake(&pSowMove);
            Soft_Motor_brake(&pSowSeed);
            break;
    }

    return;
    //Main_RunSowTask();
    //Soft_Motor_set_pwm(&qSowSeed, 5);      // 正转   逆时针  （档上）
    //Soft_Motor_set_pwm(&qSowMove, 5);      // 正转  向下
    //Soft_Motor_set_pwm(&pSowMove, 5);        // 正转    向下
    //Soft_Motor_set_pwm(&pSowSeed, 5);      // 正转  逆时针 （挡住上）


#elif TRACK_ROTATE_TEST     // track rotate and extend when stable; extend waiting when unstable
    static uint8_t test_step = 0;
    static uint32_t test_time = 0;
    if (test_step == 0) {
        test_time = TCM_Readtimer6count();
        MotorPidController_Reset(&qRotate);
        MotorPidController_Reset(&qExtend);
        test_step = 1;
    }

    if (test_step == 1) {
        step = 11;
        track_state_test = VisionFilter_l.state;
        track_target_in_limit_test = VisionFilter_l.target_in_limit;
        track_encoder_togo_test = VisionFilter_l.delta_encoder;
        track_can_extend_test =
            (track_state_test == 2U) &&
            (track_target_in_limit_test != 0U) &&
            (abs((int)track_encoder_togo_test) < 200);
        track_is_unstable_test =
            (track_state_test == 2U) &&
            (abs((int)track_encoder_togo_test) > 800);
        handle_palmotor(VisionFilter_l.state, &qRotate, &qExtend,
                        VisionFilter_l.delta_encoder,
                        VisionFilter_l.error_speed_encoder,
                        VisionFilter_l.target_in_limit,
                        1000, 2000, 80, 2000, 50000); // stable : 1000en 20ms; unstable : 2000en 500ms
        track_rotate_pwm_test = qRotate.m_motor.pwm;
        track_extend_pwm_test = qExtend.m_motor.pwm;
        track_kp_term_test = qRotate.kp_term;
        track_ki_term_test = qRotate.ki_term;
        track_kd_term_test = qRotate.kd_term;
        if ((int32_t)(TCM_Readtimer6count() - test_time) > 50000000) {
            MotorPidController_Reset(&qRotate);
            MotorPidController_Reset(&qExtend);
            test_step = 2;
        }
    } else {
        step = 12;
        set_orign(&qRotate);
        set_orign(&qExtend);
    }
    return;
#elif ENCODER_READ_TEST
    step = 20;
    encoder_pRotate_test = MotorPidController_readencoder(&pRotate);// or: pRotate.m_encoder.count
    encoder_pExtend_test = MotorPidController_readencoder(&pExtend);
    encoder_qRotate_test = MotorPidController_readencoder(&qRotate);
    encoder_qExtend_test = MotorPidController_readencoder(&qExtend);
    ccr = TIM4->CCR3;
    MotorPidController_setpwm(&qRotate, 10);
    MotorPidController_setpwm(&qExtend, 100);
    return;
#else
//none
#endif
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */


  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  User_RegisterTimers();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM6_Init();
  MX_UART4_Init();
  MX_TIM9_Init();
  MX_UART7_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_TIM7_Init();
  /* USER CODE BEGIN 2 */
  MotorPidController_ConfigApply(&qExtend,&enc_cfg1,&motor_cfg1,&ctrl_cfg1);
  MotorPidController_ConfigApply(&qRotate,&enc_cfg2,&motor_cfg2,&ctrl_cfg2);
  MotorPidController_ConfigApply(&qOpenmv,&enc_cfg_mv_l,&motor_cfg_mv_l,&ctrl_cfg_mv);

  MotorPidController_ConfigApply(&pExtend,&enc_cfg11,&motor_cfg11,&ctrl_cfg1);
  MotorPidController_ConfigApply(&pRotate,&enc_cfg22,&motor_cfg22,&ctrl_cfg2);
  MotorPidController_ConfigApply(&pOpenmv,&enc_cfg_mv_r,&motor_cfg_mv_r,&ctrl_cfg_mv);

  Soft_Motor_ConfigApply(&qSowMove,&soft_motor_cfg_1_l);
  Soft_Motor_ConfigApply(&qSowSeed,&soft_motor_cfg_2_l);
  Soft_Motor_ConfigApply(&pSowMove,&soft_motor_cfg_1_r);
  Soft_Motor_ConfigApply(&pSowSeed,&soft_motor_cfg_2_r);
  //MotorPidController_ConfigApply(&pExtend,&enc_cfg11,&motor_cfg11,&ctrl_cfg1);
  //MotorPidController_ConfigApply(&pRotate,&enc_cfg22,&motor_cfg22,&ctrl_cfg2);

  // UART receive is started in CommTask after the frame queue is created.

  HAL_TIM_Base_Start_IT(&htim6);
  HAL_TIM_Base_Start_IT(&htim7); // software_motor timer

  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); 放在init中了
  //User_Encoder_IC_Start();


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in cmsis_os2.c) */
  MX_FREERTOS_Init();

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }


  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static int16_t read_i16_le(const uint8_t *p)
{
	return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

static int8_t clamp_i8(int32_t value)
{
    if (value > 127) return 127;
    if (value < -128) return -128;
    return (int8_t)value;
}

static int16_t clamp_i16(int32_t value)
{
    if (value > 32767) return 32767;
    if (value < -32768) return -32768;
    return (int16_t)value;
}

static float read_f32_le(const uint8_t *p)
{
	uint32_t raw = ((uint32_t)p[0]) |
	               ((uint32_t)p[1] << 8) |
	               ((uint32_t)p[2] << 16) |
	               ((uint32_t)p[3] << 24);
	float value = 0.0f;
	memcpy(&value, &raw, sizeof(value));
	return value;
}

static void UartFrame_PostFromIsr(UartFrameSource_t source, const uint8_t *data, uint8_t len)
{
    if ((uart_frame_queue == NULL) || (len > UART_FRAME_MAX_LEN)) {
        return;
    }

    UartFrameMsg_t msg;
    msg.source = source;
    msg.len = len;
    memcpy(msg.data, data, len);
    (void)osMessageQueuePut(uart_frame_queue, &msg, 0U, 0U);
}

void StartReceive(UART_HandleTypeDef *huart) {
	if (huart->Instance == UART4) {
		HAL_UART_Receive_IT(&huart4, &uart_rx_buf[rx_head], 1);
	}
	else if(huart->Instance == UART7){
		HAL_UART_Receive_IT(&huart7, &uart_rx_buf_pal[rx_head_pal], 1);
	}
	else if(huart->Instance == USART1){
		HAL_UART_Receive_IT(&huart1, &uart_rx_buf_duino[rx_head_duino], 1);
	}
	else if(huart->Instance == USART2){
		HAL_UART_Receive_IT(&huart2, &uart_rx_buf_mv_l[rx_head_mv_l], 1);
	}
	else if(huart->Instance == USART3){
		HAL_UART_Receive_IT(&huart3, &uart_rx_buf_mv_r[rx_head_mv_r], 1);
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {

    if (huart->Instance == UART4) {//p10.5i11d12n  蓝牙

        if (uart_rx_buf[rx_head] == 'n')
        {
        	uart_rx_buf[rx_head] = '\0';   // 字符串结束符
            UartFrame_PostFromIsr(UART_FRAME_BT, uart_rx_buf, (uint8_t)(rx_head + 1U));
			rx_head = 0;
        }
        else if (uart_rx_buf[rx_head] == 'r')
		{
			reset_mcu = 1;
		}
        else
        {
        	rx_head = (rx_head + 1) % RX_BUF_SIZE;
        }

        StartReceive(&huart4);
    }
    else if(huart->Instance == UART7){ // 12-byte Pi: flags, L state/offset/vx, R state/offset/vx, tail
        if (rx_head_pal == PAL_RX_FRAME_LEN - 1U) {
            if (uart_rx_buf_pal[PAL_RX_FRAME_LEN - 1U] == PAL_PACKET_TAIL) {
                UartFrame_PostFromIsr(UART_FRAME_PAL, uart_rx_buf_pal, PAL_RX_FRAME_LEN);
                rx_head_pal = 0;
            } else {
                // Drop one byte and keep sliding until a 12-byte Pi frame tail is aligned.
                memmove(&uart_rx_buf_pal[0], &uart_rx_buf_pal[1], PAL_RX_FRAME_LEN - 1U);
                rx_head_pal = PAL_RX_FRAME_LEN - 1U;
            }
        }
        else{
            rx_head_pal = (rx_head_pal + 1) % RX_BUF_SIZE;
        }
        StartReceive(&huart7);
    }

    else if(huart->Instance == USART2){ // BhbBB 6-byte openmv-l: stable, dx, dy, in_window, tail
        if (rx_head_mv_l == MV_FRAME_LEN - 1U) {
            if (uart_rx_buf_mv_l[MV_FRAME_LEN - 1U] == 0xAA) {
                UartFrame_PostFromIsr(UART_FRAME_MV_L, uart_rx_buf_mv_l, MV_FRAME_LEN);
                rx_head_mv_l = 0;
            } else {
                // Fixed-size window but invalid tail: drop head and shift left.
                memmove(&uart_rx_buf_mv_l[0], &uart_rx_buf_mv_l[1], MV_FRAME_LEN - 1U);
                rx_head_mv_l = MV_FRAME_LEN - 1U;
            }
        } else {
		    rx_head_mv_l = (rx_head_mv_l + 1) % RX_BUF_SIZE;
        }
		StartReceive(&huart2);
    }
    else if(huart->Instance == USART3){ // BhbBB 6-byte openmv-r: stable, dx, dy, in_window, tail

        if (rx_head_mv_r == MV_FRAME_LEN - 1U) {
            if (uart_rx_buf_mv_r[MV_FRAME_LEN - 1U] == 0xAA) {
                UartFrame_PostFromIsr(UART_FRAME_MV_R, uart_rx_buf_mv_r, MV_FRAME_LEN);
                rx_head_mv_r = 0;

            } else {
                // Fixed-size window but invalid tail: drop head and shift left.
                memmove(&uart_rx_buf_mv_r[0], &uart_rx_buf_mv_r[1], MV_FRAME_LEN - 1U);
                rx_head_mv_r = MV_FRAME_LEN - 1U;
            }
        } else {
		    rx_head_mv_r = (rx_head_mv_r + 1) % RX_BUF_SIZE;
        }
		StartReceive(&huart3);
    }
    else if(huart->Instance == USART1){

    	if (uart_rx_buf_duino[rx_head_duino] == 0xAA && rx_head_duino==5)
		{
            UartFrame_PostFromIsr(UART_FRAME_DUINO, uart_rx_buf_duino, DUINO_FRAME_LEN);
			rx_head_duino = 0;

		}
		else{
		rx_head_duino = (rx_head_duino + 1) % RX_BUF_SIZE;
		}
		StartReceive(&huart1);  // 立即准备接收下一个字?
    }
}

static void PiStats_RequestSend(void)
{
    pi_stats_tx_pending = 1U;
}

static void PiStats_Pack(uint8_t *buf)
{
    uint8_t checksum = 0U;

    // STM32 -> Pi: 0x55, total, empty, green, yellow, left_type, right_type, seq, xor, 0xAA.
    buf[0] = PAL_TX_FRAME_HEAD;
    buf[1] = plant_all.total_plants;
    buf[2] = plant_all.type_0_count;
    buf[3] = plant_all.type_1_count;
    buf[4] = plant_all.type_2_count;
    buf[5] = wj_l.type_now;
    buf[6] = wj_r.type_now;
    buf[7] = pi_stats_tx_seq++;

    for (uint8_t i = 0U; i < 8U; i++) {
        checksum ^= buf[i];
    }
    buf[8] = checksum;
    buf[9] = PAL_PACKET_TAIL;
}

static void PiStats_ServiceTx(void)
{
    if (!pi_stats_tx_pending || uart7_tx_busy) {
        return;
    }

    PiStats_Pack(uart7_tx_buf);
    if (HAL_UART_Transmit_IT(&huart7, uart7_tx_buf, PAL_TX_FRAME_LEN) == HAL_OK) {
        uart7_tx_busy = 1U;
        pi_stats_tx_pending = 0U;
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART7) {
        uart7_tx_busy = 0U;
    }
}

static void Comm_StartReceiveAll(void)
{
    rx_head = 0;
    rx_head_pal = 0;
    rx_head_duino = 0;
    rx_head_mv_l = 0;
    rx_head_mv_r = 0;
    //StartReceive(&huart4); // blue
    StartReceive(&huart7); // pal
    StartReceive(&huart1); // duino
    StartReceive(&huart2); // mv_l
    StartReceive(&huart3); // mv_r
}

static void UartFrame_Parse(const UartFrameMsg_t *msg)
{
    int32_t lock;
    switch (msg->source) {
    case UART_FRAME_BT:
        Parse_CommandLine((char *)msg->data);
        break;
    case UART_FRAME_PAL:
        if (msg->len == PAL_RX_FRAME_LEN) {
            if (msg->data[PAL_RX_FRAME_LEN - 1U] != PAL_PACKET_TAIL) {
                break;
            }

            uint8_t flags = msg->data[0];
            uint8_t state_l = msg->data[1];
            int16_t offset_l = clamp_i16((int32_t)read_i16_le(&msg->data[2]) + PAL_LEFT_OFFSET_BIAS);
            int16_t velocity_x_l = read_i16_le(&msg->data[4]);
            uint8_t state_r = msg->data[6];
            int16_t offset_r = clamp_i16((int32_t)read_i16_le(&msg->data[7]) + PAL_RIGHT_OFFSET_BIAS);
            int16_t velocity_x_r = read_i16_le(&msg->data[9]);

            int32_t delta_encoder_l = 0;
            float error_speed_encoder_l = 0.0f;
            uint8_t target_in_limit_l = 0U;
            int32_t delta_encoder_r = 0;
            float error_speed_encoder_r = 0.0f;
            uint8_t target_in_limit_r = 0U;

            VisionTracking_ConvertTarget(
                &qRotate, offset_l, velocity_x_l,
                &delta_encoder_l, &error_speed_encoder_l, &target_in_limit_l);
            VisionTracking_ConvertTarget(
                &pRotate, offset_r, velocity_x_r,
                &delta_encoder_r, &error_speed_encoder_r, &target_in_limit_r);

            lock = osKernelLock();
            if (flags & PAL_LEFT_NEW_FRAME_FLAG) {
                VisionFrame_l_seq++;
            }
            if (flags & PAL_RIGHT_NEW_FRAME_FLAG) {
                VisionFrame_r_seq++;
            }
            VisionTracking_FilterSet(&VisionFilter_l, state_l,
                                     offset_l, velocity_x_l, delta_encoder_l,
                                     error_speed_encoder_l, target_in_limit_l,
                                     VisionFrame_l_seq);
            VisionTracking_FilterSet(&VisionFilter_r, state_r,
                                     offset_r, velocity_x_r, delta_encoder_r,
                                     error_speed_encoder_r, target_in_limit_r,
                                     VisionFrame_r_seq);
            (void)osKernelRestoreLock(lock);
            PiStats_RequestSend();
        }
        break;
    case UART_FRAME_DUINO:
        if (msg->len == DUINO_FRAME_LEN) {
            lock = osKernelLock();
            ArduinoWorkCmd.is_enable = msg->data[0];
            ArduinoWorkCmd.car_speed = read_f32_le(&msg->data[1]);
            ArduinoWorkCmd.tail = msg->data[5];
            (void)osKernelRestoreLock(lock);
        }
        break;
    case UART_FRAME_MV_R:
        if (msg->len == MV_FRAME_LEN) {
            int16_t offset_x = read_i16_le(&msg->data[1]);
            int8_t offset_y = clamp_i8((int8_t)msg->data[3] + MV_RIGHT_OFFSET_Y_BIAS);
            lock = osKernelLock();
            OpenMV_FilterSet(&MVFilter_r, offset_x, offset_y, msg->data[0], msg->data[4]);
            (void)osKernelRestoreLock(lock);
        }
        break;
    case UART_FRAME_MV_L:
        if (msg->len == MV_FRAME_LEN) {
            int16_t offset_x = read_i16_le(&msg->data[1]);
            int8_t offset_y = clamp_i8((int8_t)msg->data[3] + MV_LEFT_OFFSET_Y_BIAS);
            lock = osKernelLock();
            OpenMV_FilterSet(&MVFilter_l, offset_x, offset_y, msg->data[0], msg->data[4]);
            (void)osKernelRestoreLock(lock);
        }
        break;
    default:
        break;
    }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{

}
volatile int32_t last_loop_time = 0;
volatile int32_t motion_loop_dt_ticks = 0;
volatile int32_t motion_loop_dt_us = 0;
volatile int32_t motion_loop_dt_max_ticks = 0;
volatile int32_t motion_loop_dt_max_us = 0;
volatile int32_t loop_count = 0;

void StartMotionTask(void *argument)
{
    (void)argument;
    uint32_t next_wake_tick = osKernelGetTickCount();
    int32_t prev_loop_time = 0;

    for (;;) {
        if (reset_mcu) {
            reset_mcu = 0;
            NVIC_SystemReset();
        }
        int32_t now_loop_time = TCM_Readtimer6count();
        if (prev_loop_time != 0) {
            int32_t dt_ticks = now_loop_time - prev_loop_time;
            motion_loop_dt_ticks = dt_ticks;
            motion_loop_dt_us = dt_ticks * 10;
            if (dt_ticks > motion_loop_dt_max_ticks) {
                motion_loop_dt_max_ticks = dt_ticks;
                motion_loop_dt_max_us = dt_ticks * 10;
            }
        }
        prev_loop_time = now_loop_time;
        last_loop_time = now_loop_time;
        VisionTracking_FilterUpdate();
        Main_RunNormalLogic();
        loop_count++;


        next_wake_tick += MOTION_TASK_PERIOD_TICKS;
        if (osDelayUntil(next_wake_tick) != osOK) {
            next_wake_tick = osKernelGetTickCount() + MOTION_TASK_PERIOD_TICKS;
            osDelayUntil(next_wake_tick);
        }
    }
}

void StartTelemetryTask(void *argument)
{
    (void)argument;
    for (;;) {
        PiStats_ServiceTx();
        osDelay(TELEMETRY_TASK_PERIOD_TICKS);
    }
}

void StartCommTask(void *argument)
{
    (void)argument;
    UartFrameMsg_t msg;

    uart_frame_queue = osMessageQueueNew(UART_FRAME_QUEUE_DEPTH, sizeof(UartFrameMsg_t), NULL);
    if (uart_frame_queue == NULL) {
        Error_Handler();
    }

    Comm_StartReceiveAll();

    for (;;) {
        if (osMessageQueueGet(uart_frame_queue, &msg, NULL, osWaitForever) == osOK) {
            UartFrame_Parse(&msg);
        }
    }
}

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM6) {
      QEncoder_updatespeed(&qExtend.m_encoder);
      QEncoder_updatespeed(&qRotate.m_encoder);
      QEncoder_updatespeed(&qOpenmv.m_encoder);
      QEncoder_updatespeed(&pExtend.m_encoder);
      QEncoder_updatespeed(&pRotate.m_encoder);
      QEncoder_updatespeed(&pOpenmv.m_encoder);
      TCM_AddTimer6Overflow(2000);
  }
  else if (htim->Instance == TIM7) {
      Soft_Motor_UpdatePWM(&qSowMove);
      Soft_Motor_UpdatePWM(&qSowSeed);
      Soft_Motor_UpdatePWM(&pSowMove);
      Soft_Motor_UpdatePWM(&pSowSeed);
  }

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
