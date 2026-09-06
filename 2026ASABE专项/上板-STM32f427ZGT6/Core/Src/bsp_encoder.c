#include "bsp_encoder.h"
static const int8_t qtab_[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

static EncoderSlot slots_[TIMER_COUNT][2];  // 14个timer，每个2个slot


/* -------------------- 辅助函数 -------------------- */
uint32_t Getmask_Flag(ChannelIndex CH){
    switch(CH){
        case CH1: return TIM_FLAG_CC1;
        case CH2: return TIM_FLAG_CC2;
        case CH3: return TIM_FLAG_CC3;
        case CH4: return TIM_FLAG_CC4;
        default: return 0;
    }
}
uint32_t Getmask_Source(ChannelIndex CH){
    switch(CH){
        case CH1: return TIM_IT_CC1;
        case CH2: return TIM_IT_CC2;
        case CH3: return TIM_IT_CC3;
        case CH4: return TIM_IT_CC4;
        default: return 0;
    }
}
// 在指定组中查找空闲槽位
int findFreeSlot(Timer_ID g) {
    if (g > 14) return -1;

    for (uint8_t i = 0; i < 2; i++) {
        if (!slots_[g][i].inUse) {
            return i;
        }
    }
    return -1; // 没有空闲槽位
}
/* -------------------- 编码器专用捕获中断 -------------------- */
//放在HAL_TIM_IRQHandler 遍历编码器数组（2次）
//需要htim 和timerid
//好处一步到位清除中断标记  
//坏处 不清完会一直卡住 且不报错（法一只开对应位置的中断使能 法二全都清除）
void _HAL_TIM_IRQHandler(Timer_ID id){
    TIM_HandleTypeDef* htim = TCM_GetTimerHandle(id);
    if (!htim || !htim->Instance) return;
    uint32_t itsource = htim->Instance->DIER ;
    uint32_t itflag = htim->Instance->SR ;
    
    
    for (int i = 0; i < 2; i++) {
        EncoderSlot *s = &slots_[id][i];
        if (!s->inUse) continue;
        if( (itsource & s->source) && (itflag & s->flag) ){
            __HAL_TIM_CLEAR_FLAG(htim, s->flag);
            s->encoder->handleChange(s->encoder);
            
        }
    }
    __HAL_TIM_CLEAR_FLAG(htim, TIM_FLAG_CC1 | TIM_FLAG_CC2 | TIM_FLAG_CC3 | TIM_FLAG_CC4);

    
}
/* -------------------- 编码器专用捕获中断(花费时间更多 but 空间更少) -------------------- */
//放在HAL_TIM_IC_CaptureCallback 遍历编码器实例（n次）
//......未完  感觉不如上面
void _HAL_TIM_IRQHandler2(QEncoder* e, TIM_HandleTypeDef* htim, HAL_TIM_ActiveChannel ch){
    
}

/* -------------------- 编码器专用捕获中断handle函数 -------------------- */
void QEncoder_HandleChange(QEncoder* e)
{   
    uint8_t a = (HAL_GPIO_ReadPin(e->port_a, e->pin_a) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t b = (HAL_GPIO_ReadPin(e->port_b, e->pin_b) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t now = a<<1 | b;                
    uint8_t idx = ((e->prev << 2) | now) & 0x0F;
    e->cnt += qtab_[idx];
    e->prev = now;
    if (e->samplespeed == 1) 
    {
        e->samplespeed = 0;
        int32_t CurretTimer6 = TCM_Readtimer6count();
        float deltaTime = (CurretTimer6 - e->timer6lastcount) * 0.01f; // 0.01ms tick
        if (deltaTime <= 0.01f) deltaTime = 0.01f; // 避免除以0

        e->speed_enc = (float)(e->cnt - e->encoderlastcount) / deltaTime * 1000.0f; 
        e->encoderlastcount = e->cnt;
        e->timer6lastcount = CurretTimer6;
    }
}
/* -------------------- 编码器专用sampled & update -------------------- */
void QEncoder_updatespeed(QEncoder* e)
{
  if (e->samplespeed == 0) {
    e->samplespeed = 1;
  } else {
    int32_t CurretTimer6 = TCM_Readtimer6count();
    float deltaTime = (CurretTimer6 - e->timer6lastcount) * 0.01f; // 0.01ms tick
    if (deltaTime >= 90.0f) e->speed_enc = 0; // 长时间没更新，认为速度为0
  }
}
/* -------------------- 编码器初始化 -------------------- */
void QEncoder_Init(QEncoder* e, Timer_ID id,  ChannelIndex CH1, ChannelIndex CH2){
    int n = findFreeSlot(id);
    if (n < 0) return;
    TIM_HandleTypeDef* htim = TCM_GetTimerHandle(id);
    if (!htim || !htim->Instance) return ;
    slots_[id][n].inUse = 1;
    slots_[id][n].encoder = e;
    slots_[id][n].flag = Getmask_Flag(CH1) | Getmask_Flag(CH2);
    slots_[id][n].source = Getmask_Source(CH1) | Getmask_Source(CH2);
    const PinConfig* pa = TCM_GetPinConfig(id, CH1);
    const PinConfig* pb = TCM_GetPinConfig(id, CH2);
    if (!pa || !pb) return;

    e->port_a = pa->port;
    e->pin_a = pa->pin;
    e->port_b = pb->port;
    e->pin_b = pb->pin;
    e->handleChange = QEncoder_HandleChange;
    HAL_TIM_IC_Start_IT(htim, TCM_HAL_CH[CH1]);
    HAL_TIM_IC_Start_IT(htim, TCM_HAL_CH[CH2]);
}
QEnc_Status QEncoder_ConfigApply(QEncoder* e, const QEncoder_Config* cfg)
{
    if (!e || !cfg) return QENC_ERR_NULL_PTR;
    if ((uint32_t)cfg->id >= TIMER_COUNT) return QENC_ERR_BAD_TIMER;

    TIM_HandleTypeDef* htim = TCM_GetTimerHandle(cfg->id);
    if (!htim || !htim->Instance) return QENC_ERR_BAD_TIMER;

    int n = findFreeSlot(cfg->id);
    if (n < 0) return QENC_ERR_NO_SLOT;

    const PinConfig* pa = TCM_GetPinConfig(cfg->id, cfg->chA);
    const PinConfig* pb = TCM_GetPinConfig(cfg->id, cfg->chB);
    if (!pa || !pb) return QENC_ERR_BAD_PINCFG;

    // 先把 encoder 配好
    e->port_a = pa->port; e->pin_a = pa->pin;
    e->port_b = pb->port; e->pin_b = pb->pin;
    e->cnt = 0;
    e->handleChange = QEncoder_HandleChange;



    // 初始化 prev 为当前电平（避免第一下跳变算错）
    
    uint8_t a = (HAL_GPIO_ReadPin(e->port_a, e->pin_a) == GPIO_PIN_SET) ? 1 : 0;
    uint8_t b = (HAL_GPIO_ReadPin(e->port_b, e->pin_b) == GPIO_PIN_SET) ? 1 : 0;
    e->prev = (uint8_t)((a << 1) | b);
    

    // 再注册 slot（最后一步，避免中途失败留下脏 slot）
    EncoderSlot* s = &slots_[cfg->id][n];
    s->encoder = e;
    s->flag    = Getmask_Flag(cfg->chA) | Getmask_Flag(cfg->chB);
    s->source  = Getmask_Source(cfg->chA) | Getmask_Source(cfg->chB);
    s->inUse   = 1;

    if (HAL_TIM_IC_Start_IT(htim, TCM_HAL_CH[cfg->chA]) != HAL_OK) {
    return QENC_ERR_BAD_TIMER;
    }
    if (HAL_TIM_IC_Start_IT(htim, TCM_HAL_CH[cfg->chB]) != HAL_OK) {
        HAL_TIM_IC_Stop_IT(htim, TCM_HAL_CH[cfg->chA]);
        return QENC_ERR_BAD_TIMER;
    }



    return QENC_OK;
}
int32_t QEncoder_Read(QEncoder* e)
{
    return e ? e->cnt : 0;
}