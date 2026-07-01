#include "crt_chassis.h"

/**
 * @brief 底盘初始化
 *
 * @param __Chassis_Control_Type 底盘控制方式, 默认舵轮方式
 * @param __Speed 底盘速度限制最大值
 */
void Class_Omni_Chassis::Init(float __Velocity_X_Max, float __Velocity_Y_Max, float __Omega_Max)
{
    Velocity_X_Max = __Velocity_X_Max;
    Velocity_Y_Max = __Velocity_Y_Max;
    Omega_Max = __Omega_Max;

    // 斜坡函数加减速速度X  控制周期1ms
    Slope_Velocity_X.Init(0.006f, 0.012f, Slope_First_REAL);
    // 斜坡函数加减速速度Y  控制周期1ms
    Slope_Velocity_Y.Init(0.006f, 0.012f, Slope_First_REAL);
    // 斜坡函数加减速角速度
    Slope_Omega.Init(0.010f, 0.010f, Slope_First_REAL);

    // 底盘速度xPID, 输出摩擦力
    PID_Velocity_X.Init(120.0f, 0.0f, 0.0f, 0.0f, 0.18f, 50.0f, 0.002f);

    // 底盘速度yPID, 输出摩擦力
    PID_Velocity_Y.Init(120.0f, 0.0f, 0.0f, 0.0f, 0.18f, 50.0f, 0.002f);

    // 底盘角速度PID, 输出扭矩
    PID_Omega.Init(50.0f, 0.0f, 0.0f, 0.0f, 0.01f, 50.0f, 0.002f);

    // 轮向电机ID初始化
    Motor_Wheel[0].Init(&hfdcan1, Motor_DJI_ID_0x201);
    Motor_Wheel[1].Init(&hfdcan1, Motor_DJI_ID_0x202);
    Motor_Wheel[2].Init(&hfdcan1, Motor_DJI_ID_0x203);
}

/**
 * @brief 全向轮运动学逆解算
 *
 */
void Class_Omni_Chassis::Kinematics_Inverse_Resolution()
{
    switch (Chassis_Control_Type)
    {
    case (Chassis_Control_Type_DISABLE):
    {
        for (int i = 0; i < 3; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
            Motor_Wheel[i].PID_Angle.Set_Integral_Error(0.0f);
            Motor_Wheel[i].Set_Target_Omega(0.0f);
            Motor_Wheel[i].Set_Out(0.0f);
        }
    }
    break;

    case (Chassis_Control_Type_FLLOW):
    {
        // 底盘四电机模式配置
        for (int i = 0; i < 3; i++)
        {
            Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        }
        // 底盘限速
        if (Velocity_X_Max != 0)
            Math_Constrain(&Target_Velocity_X, -Velocity_X_Max, Velocity_X_Max);

        if (Velocity_Y_Max != 0)
            Math_Constrain(&Target_Velocity_Y, -Velocity_Y_Max, Velocity_Y_Max);

        if (Omega_Max != 0)
            Math_Constrain(&Target_Omega, -Omega_Max, Omega_Max);

#ifdef SPEED_SLOPE
        // 速度换算，逆运动学分解
        float motor1_temp_linear_vel = Slope_Velocity_Y.Get_Out() + Slope_Omega.Get_Out() * WHEEL_TO_CORE_DISTANCE;
        float motor2_temp_linear_vel = -Slope_Velocity_X.Get_Out() * 0.5f * SQRT3 - Slope_Velocity_Y.Get_Out() * 0.5f + Slope_Omega.Get_Out() * WHEEL_TO_CORE_DISTANCE;
        float motor3_temp_linear_vel = Slope_Velocity_X.Get_Out() * 0.5f * SQRT3 - Slope_Velocity_Y.Get_Out() * 0.5f + Slope_Omega.Get_Out() * WHEEL_TO_CORE_DISTANCE;
#else
        // 速度换算，逆运动学分解
        float motor1_temp_linear_vel = Target_Velocity_X + Target_Omega * WHEEL_TO_CORE_DISTANCE;
        float motor2_temp_linear_vel = -Target_Velocity_X * 0.5f + Target_Velocity_Y * 0.5f * SQRT3 + Target_Omega * WHEEL_TO_CORE_DISTANCE;
        float motor3_temp_linear_vel = -Target_Velocity_X * 0.5f - Target_Velocity_Y * 0.5f * SQRT3 + Target_Omega * WHEEL_TO_CORE_DISTANCE;
#endif
        // 线速度 cm/s  转角速度  RAD
        float motor1_temp_rad = motor1_temp_linear_vel * VEL2RAD;
        float motor2_temp_rad = motor2_temp_linear_vel * VEL2RAD;
        float motor3_temp_rad = motor3_temp_linear_vel * VEL2RAD;

        // 角速度*减速比  设定目标 直接给到电机输出轴
        Motor_Wheel[0].Set_Target_Omega(-motor1_temp_rad);
        Motor_Wheel[1].Set_Target_Omega(-motor2_temp_rad);
        Motor_Wheel[2].Set_Target_Omega(-motor3_temp_rad);

        // test
        //  Motor_Wheel[0].Set_Target_Omega_Radian(temp_test_1);
        //  Motor_Wheel[1].Set_Target_Omega_Radian(temp_test_2);
        //  Motor_Wheel[2].Set_Target_Omega_Radian(temp_test_3);
        //  Motor_Wheel[3].Set_Target_Omega_Radian(temp_test_4);
    }
    break;
    }
    // 各个电机具体PID
    for (int i = 0; i < 3; i++)
    {
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }
}

/**
 * @brief 自身解算
 *
 */
void Class_Omni_Chassis::Self_Resolution()
{
    float temp_velocity_x_now = 0.0f, temp_velocity_y_now = 0.0f, temp_omega_now = 0.0f;
    float Now_Wheel_Omega[3] = {0.0f};
    for (int i = 0; i < 3; i++)
    {
        Now_Wheel_Omega[i] = - Motor_Wheel[i].Get_Now_Omega();
    }

    temp_velocity_x_now = (Now_Wheel_Omega[2] - Now_Wheel_Omega[1]) * SQRT3 * WHEEL_DIAMETER / 6.0f;
    temp_velocity_y_now = (2.0f * Now_Wheel_Omega[0] - Now_Wheel_Omega[1] - Now_Wheel_Omega[2]) * WHEEL_DIAMETER / 6.0f;
    temp_omega_now = (Now_Wheel_Omega[0] + Now_Wheel_Omega[1] + Now_Wheel_Omega[2]) * WHEEL_DIAMETER / (6.0f * WHEEL_TO_CORE_DISTANCE);

    Set_Now_Velocity_X(temp_velocity_x_now);
    Set_Now_Velocity_Y(temp_velocity_y_now);
    Set_Now_Omega(temp_omega_now);
}

/**
 * @brief TIM定时器中断计算回调函数
 *
 */
void Class_Omni_Chassis::TIM_Calculate_PeriodElapsedCallback(Enum_Sprint_Status __Sprint_Status)
{
    // 斜坡函数计算用于速度解算初始值获取
    Slope_Velocity_X.Set_Now_Real(Now_Velocity_X);
    Slope_Velocity_X.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_X.Set_Target(Target_Velocity_X);

    Slope_Velocity_Y.Set_Now_Real(Now_Velocity_Y);
    Slope_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();
    Slope_Velocity_Y.Set_Target(Target_Velocity_Y);

    Slope_Omega.Set_Now_Real(Now_Omega);

    Slope_Omega.TIM_Calculate_PeriodElapsedCallback();
    Slope_Omega.Set_Target(Target_Omega);

    Self_Resolution(); // 自身解算

    Output_To_Dynamics(); // 计算速度环PID

    Dynamics_Inverse_Resolution(); // 分配力矩并前馈

    Output_To_Motor(); // 最终下发至电机
}

/**
 * @brief 100ms定时器存活回调函数
 *
 */
void Class_Omni_Chassis::TIM_100ms_Alive_PeriodElapsedCallback()
{
    Motor_Wheel[0].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Wheel[1].TIM_100ms_Alive_PeriodElapsedCallback();
    Motor_Wheel[2].TIM_100ms_Alive_PeriodElapsedCallback();
}

/**
 * @brief 动力学逆解算
 *
 */
void Class_Omni_Chassis::Dynamics_Inverse_Resolution()
{
    float force_x = 0.0f, force_y = 0.0f, torque_omega = 0.0f;

    force_x = PID_Velocity_X.Get_Out();
    force_y = PID_Velocity_Y.Get_Out();
    torque_omega = PID_Omega.Get_Out();

    // 每个轮的扭力
    float force_wheel[3] = {0.0f};
    for (int i = 0; i < 3; i++)
    {
        float theta = i * 2.0f * 3.1415926f / 3.0f;
        force_wheel[i] = -force_x * arm_sin_f32(theta) + force_y * arm_cos_f32(theta) + torque_omega / (WHEEL_TO_CORE_DISTANCE * 3.0f);
    }
    for (int i = 0; i < 3; i++)
    {
        Target_Wheel_Current[i] = DIRECTION_SIGN * force_wheel[i] * WHEEL_DIAMETER / 2.0f;

        // // 动摩擦阻力前馈
        // if (Target_Wheel_Omega[i] > Wheel_Resistance_Omega_Threshold)
        // {
        //     Target_Wheel_Current[i] += Dynamic_Resistance_Wheel_Current[i];
        // }
        // else if (Target_Wheel_Omega[i] < -Wheel_Resistance_Omega_Threshold)
        // {
        //     Target_Wheel_Current[i] -= Dynamic_Resistance_Wheel_Current[i];
        // }
        // else
        // {
        //     Target_Wheel_Current[i] += Motor_Wheel[i].Get_Now_Omega() / Wheel_Resistance_Omega_Threshold * Dynamic_Resistance_Wheel_Current[i];
        // }

        // 低电流前馈控制模式
        if (Math_Abs(Target_Wheel_Current[i]) < Low_Current_Deadzone)
        {
            Target_Wheel_Current[i] = 0.0f;
        }
        else if (Math_Abs(Target_Wheel_Current[i]) < Low_Current_Threshold)
        {
            // 如果电流小于阈值，添加前馈
            if (Target_Wheel_Current[i] > 0)
            {
                Target_Wheel_Current[i] += Low_Current_Feedforward[i];
            }
            else if (Target_Wheel_Current[i] < 0)
            {
                Target_Wheel_Current[i] -= Low_Current_Feedforward[i];
            }
        }
    }
    // 根据斜坡与压力进行电流限幅防止贴地打滑
    // 待定
}

/**
 * @brief 输出到动力学状态
 *
 */
void Class_Omni_Chassis::Output_To_Dynamics()
{
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
            // 底盘失能
            for (int i = 0; i < 3; i++)
            {
                PID_Velocity_X.Set_Integral_Error(0.0f);
                PID_Velocity_Y.Set_Integral_Error(0.0f);
                PID_Omega.Set_Integral_Error(0.0f);
            }

            break;

        case (Chassis_Control_Type_FLLOW):
            PID_Velocity_X.Set_Target(Target_Velocity_X);
            PID_Velocity_X.Set_Now(Now_Velocity_X);
            PID_Velocity_X.TIM_Calculate_PeriodElapsedCallback();

            PID_Velocity_Y.Set_Target(Target_Velocity_Y);
            PID_Velocity_Y.Set_Now(Now_Velocity_Y);
            PID_Velocity_Y.TIM_Calculate_PeriodElapsedCallback();

            PID_Omega.Set_Target(Target_Omega);
            PID_Omega.Set_Now(Now_Omega);
            PID_Omega.TIM_Calculate_PeriodElapsedCallback();

            break;

        default:
            break;
    }
}

void Class_Omni_Chassis::Output_To_Motor()
{
    switch (Chassis_Control_Type)
    {
        case (Chassis_Control_Type_DISABLE):
        {
            // 底盘失能
            for (int i = 0; i < 3; i++)
            {
                Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
                Motor_Wheel[i].Set_Target_Current(0.0f);
            }

            break;
        }
        case (Chassis_Control_Type_FLLOW):
        {
            for (int i = 0; i < 3; i++)
            {
                Motor_Wheel[i].Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
                Motor_Wheel[i].Set_Target_Current(Target_Wheel_Current[i]);
            }
            break;
        }
        default:
            break;
    }

    for (int i = 0; i < 3; i++)
    {
        Motor_Wheel[i].TIM_Calculate_PeriodElapsedCallback();
    }
}