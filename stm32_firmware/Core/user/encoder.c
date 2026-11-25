#include "encoder.h"
#define enc_sign -1 //翻转符号用
typedef struct 
{
   TIM_HandleTypeDef *htimx;
   int32_t last;//上次读数
   int32_t delta;//服务速度环
   int32_t total;//服务里程计
   float speed_rpm;//车轮角速度
   float buf[5];//滑动窗口，滤波
   uint8_t idx;//标记窗口下标
}enc;
static volatile enc ABenc[4]=
{
    {&htim2,0,0,0,0.0f,{0},0},
    {&htim3,0,0,0,0.0f,{0},0},
    {&htim4,0,0,0,0.0f,{0},0},
    {&htim5,0,0,0,0.0f,{0},0},
};
void enc_init(void)
{
    HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5,TIM_CHANNEL_ALL);
    //清0
     for (int i = 0; i<4; i++) {
        __HAL_TIM_SET_COUNTER(ABenc[i].htimx, 0);
        ABenc[i].last  = 0;
        ABenc[i].total = 0;
        ABenc[i].idx=0;
        ABenc[i].speed_rpm=0.0f;
        for(int j=0;j<5;j++)
        {
            ABenc[i].buf[j]=0;
        }
    }


}
void enc_update(void)
{
    for(int i=0;i<4;i++)
    {
        int32_t now=0;
        int32_t d=0;
        now=(int32_t)__HAL_TIM_GET_COUNTER(ABenc[i].htimx);
        d=(int16_t)(now-ABenc[i].last);//强转防溢出
        d=enc_sign*d;
        //更新
        ABenc[i].last=now;
        ABenc[i].delta=d;
        ABenc[i].total+=d;
        //滑动窗口更新
        ABenc[i].buf[ABenc[i].idx]=ABenc[i].delta;
        ABenc[i].idx=(ABenc[i].idx+1)%5;
        //滑动平均滤波
        float sum=0.0f;
        for(int j=0;j<5;j++)
        {
            sum+=ABenc[i].buf[j];
        }
        float avg=(float)sum/5.0f*K_RPM;
        //一阶低通滤波
        ABenc[i].speed_rpm=0.8f*ABenc[i].speed_rpm+0.2f*avg;
    }
}
int32_t enc_get_total(uint8_t id)
{
    return ABenc[id].total;
}
float enc_get_speed_rpm(uint8_t id)
{
    return ABenc[id].speed_rpm;
}
int32_t enc_get_delta(uint8_t id)
{
    return ABenc[id].delta;
}
