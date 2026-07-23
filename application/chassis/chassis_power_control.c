#include "chassis_power_control.h"
#include "message_center.h"
#include "arm_math.h"
#include "robot_def.h"
#include "remote_control.h"
#include "controller.h"


// extern cap_measure_t cap_measure; // capacitor data structure
// extern RC_ctrl_t rc_ctrl;
// uint8_t cap_state = 0;

static Publisher_t *chassis_power_pub;   // 底盘控制消息发布者
static Subscriber_t *chassis_feed_sub; // 底盘反馈信息订阅者
static Chassis_Power_Data_s chassis_power_send;      // 发送给底盘应用的信息,包括控制信息和UI绘制相关
static Chassis_Upload_Data_s chassis_fetch_data; // 从底盘应用接收的反馈信息信息,底盘功率枪口热量与底盘运动状态等
static uint8_t cap_state = 0; // capacitor state

static uint16_t max_power_limit = 10;
static float32_t chassis_max_power = 0;
static float input_power = 0;		 // input power from battery (referee system)
static float allocate_give_power[4]; // initial power from PID calculation
static float debug_allocate_total_power = 0;//debug监视
static float allocate_total_power = 0;
static float32_t distribut_give_power[4];
static float32_t chassis_power = 0.0f;
static float32_t chassis_power_buffer = 0.0f;

static 	float32_t toque_coefficient = 0.000002; // (20/16384)*(0.3)*(187/3591)/9.55
static 	float32_t k1 = 0.000000123;						 // k1
static 	float32_t k2 = 0.00000018;					 // k2
static 	float32_t constant = 0.725;

static PIDInstance power_buffer_pid;

/// @brief 
/// @param  
void chassis_power_control_init(void)
{
    PID_Init_Config_s pid_cfg = {
        .Kp = 0.5,//0.3
        .Ki = 0,
        .Kd = 0.3,
        .DeadBand = 0,
        .MaxOut = 20,
        .MaxOut_ = -20,
        .IntegralLimit = 5,
        .Derivative_LPF_RC =0.05,
        .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement | PID_DerivativeFilter,
    };
    PIDInit(&power_buffer_pid, &pid_cfg);
    chassis_power_pub = PubRegister("power_cmd", sizeof(Chassis_Power_Data_s));
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_Upload_Data_s));
}
static float ref_power;
float ks=0.6;
float debug_stl;//规划功率
/// @brief 
/// @param
void chassis_power_control(void)
{
    SubGetMessage(chassis_feed_sub, (void *)&chassis_fetch_data);
    allocate_total_power*=ks;
    //debug_power_up=0;
    //debug_power_down=0;
	for(size_t i = 0;i < 4; i++)
	{
        allocate_give_power[i] = toque_coefficient * chassis_fetch_data.motor_current[i] * chassis_fetch_data.motor_speed[i] + chassis_fetch_data.motor_current[i] * chassis_fetch_data.motor_current[i] * k1 + chassis_fetch_data.motor_speed[i] * k2 * chassis_fetch_data.motor_speed[i] + constant;
        allocate_total_power +=(1-ks)*fabs(allocate_give_power[i]);
	}
    ref_power=allocate_total_power+PIDCalculate(&power_buffer_pid,allocate_total_power,chassis_fetch_data.chassis_power_limit);
    ref_power=ref_power<0?0:ref_power;
    ref_power=ref_power<chassis_fetch_data.chassis_power_limit?ref_power:chassis_fetch_data.chassis_power_limit;
    //debug
    debug_stl=ref_power;
    debug_allocate_total_power = allocate_total_power;
    float32_t power_time = ref_power / allocate_total_power;
	for(size_t i = 0; i < 4; i++)
	{
        distribut_give_power[i] = ref_power/4.0;//allocate_give_power[i] * power_time;
		float32_t a = k1;
		float32_t b = chassis_fetch_data.motor_speed[i] * toque_coefficient;
		float32_t c = k2 * chassis_fetch_data.motor_speed[i] * chassis_fetch_data.motor_speed[i] + constant -  distribut_give_power[i];
        if(b * b - 4 * a * c > 0)
        {
            if((-b + sqrt(b * b - 4 * a * c)) / (2 * a) > 0)
            {
                chassis_power_send.motor_current_up[i] =(-b + sqrt(b * b - 4 * a * c)) / (2 * a)>0?(-b + sqrt(b * b - 4 * a * c)) / (2 * a):0;
                chassis_power_send.motor_current_down[i] =(-b - sqrt(b * b - 4 * a * c)) / (2 * a)<0?(-b - sqrt(b * b - 4 * a * c)) / (2 * a):0;
            }
            else
            {
                chassis_power_send.motor_current_down[i] = (-b + sqrt(b * b - 4 * a * c)) / (2 * a)<0?(-b + sqrt(b * b - 4 * a * c)) / (2 * a):0;
                chassis_power_send.motor_current_up[i] = (-b - sqrt(b * b - 4 * a * c)) / (2 * a)>0?(-b - sqrt(b * b - 4 * a * c)):0;
            }
        }
        /*
        //debug
        debug_allocate_give_power_up[i] =toque_coefficient * chassis_power_send.motor_current_up[i] * chassis_fetch_data.motor_speed[i] + chassis_power_send.motor_current_up[i] * chassis_power_send.motor_current_up[i] * k1 + chassis_fetch_data.motor_speed[i] * k2 * chassis_fetch_data.motor_speed[i] + constant;
        debug_allocate_give_power_down[i] =toque_coefficient * chassis_power_send.motor_current_down[i] * chassis_fetch_data.motor_speed[i] + chassis_power_send.motor_current_down[i] * chassis_power_send.motor_current_down[i] * k1 + chassis_fetch_data.motor_speed[i] * k2 * chassis_fetch_data.motor_speed[i] + constant;
        */
    }
    /*
    debug_stl=distribut_give_power[0]+ distribut_give_power[1]+distribut_give_power[2]+distribut_give_power[3];
    debug_power_up=debug_allocate_give_power_up[0]+debug_allocate_give_power_up[1]+debug_allocate_give_power_up[2]+debug_allocate_give_power_up[3];
    debug_power_down=debug_allocate_give_power_down[0]+debug_allocate_give_power_down[1]+debug_allocate_give_power_down[2]+debug_allocate_give_power_down[3];
    */
    chassis_power_send.chassis_power_mx = allocate_total_power;
	PubPushMessage(chassis_power_pub, (void *)&chassis_power_send);
}