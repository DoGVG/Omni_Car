#include "chasis.h"
static float vx_target, vy_target, wz_target, vx_act, vy_act, wz_act;
static float rpm_target[4];
static float spd_act[4];
static volatile float yaw_imu, yaw_enc,gyro_res;
static volatile float odom_x, odom_y;
static uint16_t still_cnt;
#define K_ERRINJECt 	0
#define CORR_MAX	0.00002f
#define STILL_WZ    0.02f
#define STILL_GZ    0.05f
#define STILL_CNTMAX   100
#define KZ	 0.0001f
/*
弧度归一化到+-PI
*/
static inline float wrap(float a)
{
    while(a>=PI)    a-=2.0f*PI;
    while(a<=-PI)    a+=2.0f*PI;
    return a;
}
/*
作用是对打滑进行限幅，避免污染yaw
*/
static inline float err_inject(float err)
{
    err*=K_ERRINJECt;
    if(err>=0.0f)  return err>CORR_MAX? CORR_MAX:err;
    if(err<0.0f)  return err<-CORR_MAX? -CORR_MAX:err;
    return 0.0f;
}
/*
静止检测,STILL_CNTMAX拍都符合才算静止
*/
static uint8_t is_still(void)
{
    if(fabsf(imu_get_gz())<=STILL_GZ&&fabsf(wz_act)<=STILL_WZ)
    {
       if(still_cnt<STILL_CNTMAX) still_cnt++;
    }
    else
    {
        still_cnt=0;
    }
    return (still_cnt>=STILL_CNTMAX);
    

}
void chasis_init(void)
{
    vx_target = vy_target = wz_target = vx_act = vy_act = wz_act = odom_x = odom_y = yaw_enc =yaw_imu=gyro_res=still_cnt= 0;
    for (int i = 0; i < 4; i++)
    {
        rpm_target[i] = 0;
    }
}

// 正解
void chasis_update(void)
{ // 读值
    float spd_all = 0;
    for (uint8_t i = 0; i < 4; i++)
    {
        spd_act[i] = enc_get_speed_rpm(i) * 2.0f * PI * R_wheel / 60.0f;
        spd_all += spd_act[i];
    }
    // 车头系获取速度
    wz_act = spd_all / 4.0f / R_chasis;
    float vx_b = (spd_act[M_RR] + spd_act[M_FR] - spd_act[M_FL] - spd_act[M_RL]) / 4.0f /K_body;
    float vy_b = (spd_act[M_FL] + spd_act[M_FR] - spd_act[M_RL] - spd_act[M_RR]) / 4.0f /K_body;
    //静止检测,获取漂移残值
    if(is_still())
    {
        gyro_res+=KZ*(imu_get_gz()-gyro_res);
    }

    // 积分获取yaw
    yaw_imu += (imu_get_gz()-gyro_res) * DT;
    yaw_enc += wz_act * DT;
    // 航向融合
    float err=wrap(yaw_enc-yaw_imu);
    yaw_imu+=err_inject(err);
    yaw_imu=wrap(yaw_imu);
    yaw_enc=wrap(yaw_enc);

    // 世界系转换
    float c = cosf(yaw_imu);
    float s = sinf(yaw_imu);
    vx_act = vx_b * c - vy_b * s;
    vy_act = vx_b * s + vy_b * c;
    //积分获取里程计
    odom_x+=vx_act*DT;
    odom_y+=vy_act*DT;
}

// 逆解
void chasis_targetrpm_cal(float vx_w, float vy_w, float wz)
{
    // 世界系向偏航后的车头系转换
    float c = cosf(yaw_imu);
    float s = sinf(yaw_imu);
    float vx_b = vx_w * c + vy_w * s;
    float vy_b = vy_w * c - vx_w * s;
    // 车头系速度解算
    rpm_target[M_FL] = (wz * R_chasis + K_body * (-vx_b+vy_b)) * 60 / (2 * PI * R_wheel);
    rpm_target[M_FR] = (wz * R_chasis + K_body * (vx_b + vy_b)) * 60 / (2 * PI * R_wheel);
    rpm_target[M_RL] = (wz * R_chasis + K_body * (-vx_b - vy_b)) * 60 / (2 * PI * R_wheel);
    rpm_target[M_RR] = (wz * R_chasis + K_body * (vx_b - vy_b)) * 60 / (2 * PI * R_wheel);
}
float chasis_get_rpm_tar(uint8_t id)
{
    return rpm_target[id];
}
float chasis_get_yaw(void)
{
    return yaw_imu;      /* rad，范围 ±π */
}
float chasis_get_yaw_enc(void)
{
    return yaw_enc;      /* 纯编码器积分，融合调试用 */
}
float chasis_get_wz(void)
{
    return wz_act;
}
float chasis_get_vx(void)
{
    return vx_act;
}
float chasis_get_vy(void)
{
    return vy_act;
}
float chasis_get_odom_x(void)
{
    return odom_x;
}
float chasis_get_odom_y(void)
{
    return odom_y;
}
void towards_test(void)
{
    chasis_targetrpm_cal(0.0f, 0, 3.0f);
    float duty[4];
    for (int i = 0; i < 4; i++)
    {
        duty[i] = rpm_target[i] / 333.0f;
    }
    for (int i = 0; i < 4; i++)
    {
        setpwm(i, duty[i]);
    }
}