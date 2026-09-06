//code building
/*
typedef enum {
    WORK_OFF = 0,     // enable=0
    WORK_ARMING,      // enable刚变1，延时准备中（全系统不动作/只保持等待）
    WORK_ACTIVE        // 延时结束，正式工作
} WorkPhase_t;

static WorkPhase_t g_work_phase = WORK_OFF;
static uint8_t g_prev_enable = 0;
static uint32_t g_work_deadline = 0;  // 延时截止时间
#define WORK_START_DELAY_MS 100000       //100 => 1ms

static uint8_t prev_enable = 0;
uint8_t curr_enable = ArduinoWorkCmd.is_enable;



void System_UpdateWorkPhase(void)
{
    uint8_t en = ArduinoWorkCmd.is_enable;
    uint32_t now = TCM_Readtimer6count();  //100tick = 1ms

    if (en == 1 && g_prev_enable == 0) {
        System_EnterWork(now);
    } else if (en == 0 && g_prev_enable == 1) {
        System_ExitWork();
    }

    g_prev_enable = en;

    // ARMING -> ACTIVE
    if (g_work_phase == WORK_ARMING) {
        if ((int32_t)(now - g_work_deadline) >= 0) {
            g_work_phase = WORK_ACTIVE;
            WindowJudge_EnterWork(&wj_l,MVpacket_l.is_plant_find);
            WindowJudge_EnterWork(&wj_r,MVpacket_r.is_plant_find)
        }
    }
}
static inline void System_EnterWork(uint32_t now_ms)
{
    g_work_phase = WORK_ARMING;
    g_work_deadline = now_ms + WORK_START_DELAY_MS;

    // 1) 所有电机控制器“清状态”（避免上一次残留）
    MotorPidController_Reset(&qExtend);
    MotorPidController_Reset(&pExtend);
    MotorPidController_Reset(&qRotate);
    MotorPidController_Reset(&pRotate);
    MotorPidController_Reset(&qOpenmv);
    MotorPidController_Reset(&pOpenmv);

    // 2) 所有模块进入等待姿态（不要开始追踪）
    set_waiting(&qExtend); set_waiting(&pExtend);
    set_waiting(&qRotate); set_waiting(&pRotate);
    set_waiting(&qOpenmv); set_waiting(&pOpenmv);

    // 3) 视觉统计/窗口也先 reset，但不立即开窗
    WindowJudge_Reset(&wj_l);
    WindowJudge_Reset(&wj_r);


}
static inline void System_ExitWork(void)
{
    g_work_phase = WORK_OFF;

    // 停止/返航
    set_orign(&qExtend); set_orign(&pExtend);
    set_orign(&qRotate); set_orign(&pRotate);
    set_orign(&qOpenmv); set_orign(&pOpenmv);

    // 清理视觉状态
    WindowJudge_Reset(&wj_l);
    WindowJudge_Reset(&wj_r);
}
void loop_main(void)
{
    System_UpdateWorkPhase();

    if (g_work_phase == WORK_OFF) {
        // OFF：返航（你现在的 else 分支）
        set_orign(&qExtend);
        set_orign(&pExtend);
        set_orign(&qRotate);
        set_orign(&pRotate);
        set_orign(&qOpenmv);
        set_orign(&pOpenmv);
        return;
    }

    if (g_work_phase == WORK_ARMING) {
        // ARMING：延时期间所有控制器保持等待，不追踪、不计数、不投种
        set_waiting(&qExtend); set_waiting(&pExtend);
        set_waiting(&qRotate); set_waiting(&pRotate);
        set_waiting(&qOpenmv); set_waiting(&pOpenmv);
        return;
    }

    if (g_work_phase == WORK_ACTIVE) {

        track_and_extend(200, 7000, 200, 800, 30000);
        openmv_extend(200,5);
        PlantClassifier_Update();   // process_packet normal path
        
        // is_should_plant();   // sow logic only in ACTIVE path
    }
    Execute_Sow_Sequence(&wj_l, &MVpacket_l, &qSowMove, &qSowSeed);
    Execute_Sow_Sequence(&wj_r, &MVpacket_r, &pSowMove, &pSowSeed);
}
    */
