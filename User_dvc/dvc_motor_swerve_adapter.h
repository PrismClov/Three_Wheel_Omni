#ifndef DVC_MOTOR_SWERVE_ADAPTER_H
#define DVC_MOTOR_SWERVE_ADAPTER_H

#include <stdint.h>
#include <math.h>

#include "dvc_motor_base.h"
#include "dvc_motor_dji.h"
#include "dvc_motor_mksesc.h"

/**
 * @brief DJI GM6020 舵向电机适配器
 *
 * 作用：
 * 把现有 Class_Motor_DJI_GM6020 适配成 Class_Motor_Base，
 * 这样可以直接接入 Class_Swerve_Module。
 *
 * 推荐用途：
 * 舵向电机，控制模式使用 MOTOR_CONTROL_MODE_POSITION。
 *
 * 注意：
 * 这里不重写 DJI 底层 CAN 和 PID，只做统一接口适配。
 * 舵向位置模式下，适配器使用外部反馈 Feedback_Position 做一层位置环，
 * 输出目标角速度给 GM6020 原有速度环。
 */
class Class_Motor_DJI_C610_Adapter : public Class_Motor_Base
{
public:
    struct Parameters
    {
        /*
         * 舵向位置外环 PID。
         * 单位：
         * error: rad
         * output: rad/s
         */
        float Position_Kp = 8.0f;
        float Position_Ki = 0.0f;
        float Position_Kd = 0.1f;

        float Position_Integral_Limit = 2.0f;
        float Position_Output_Limit = 20.0f; // rad/s

        /*
         * 是否使用外部传入的 Feedback_Position。
         * 对舵轮来说通常为 true，因为舵向建议用绝对编码器反馈。
         */
        bool Use_External_Position_Feedback = true;

        /*
         * 如果 DJI 底层启用了功率限制，可置 true。
         * 普通舵向电机建议 false。
         */
        bool Use_Power_Limit_After_Calculate = false;
    };

public:
    Class_Motor_DJI_C610_Adapter() = default;

    bool Init(
        Class_Motor_DJI_C610& motor,
        const Parameters& parameters
    );

    void Init() override;

    void Set_Control_Mode(Enum_Motor_Control_Mode mode) override;

    void Set_Target_Current(float current) override;
    void Set_Target_Speed(float speed) override;
    void Set_Target_Position(float position) override;

    void Set_Feedback_Current(float current) override;
    void Set_Feedback_Speed(float speed) override;
    void Set_Feedback_Position(float position) override;

    float Get_Current() const override;
    float Get_Speed() const override;
    float Get_Position() const override;

    float Get_Target_Current() const;
    float Get_Target_Speed() const;
    float Get_Target_Position() const;

    bool Is_Initialized() const;

    void Update_Feedback() override;
    void Calculate() override;
    void Output() override;

private:
    Class_Motor_DJI_C610* Motor = nullptr;
    Parameters Param;

    Enum_Motor_Control_Mode Control_Mode = MOTOR_CONTROL_MODE_DISABLE;

    float Target_Current = 0.0f;   // A
    float Target_Speed = 0.0f;     // rad/s
    float Target_Position = 0.0f;  // rad

    float Feedback_Current = 0.0f;   // A
    float Feedback_Speed = 0.0f;     // rad/s
    float Feedback_Position = 0.0f;  // rad

    bool Initialized = false;

    float Position_Error = 0.0f;
    float Position_Last_Error = 0.0f;
    float Position_Integral = 0.0f;

private:
    float Calculate_Position_PID(float target, float feedback);
    void Clear_Position_PID();

    static float Normalize_Angle(float angle);
    static float Limit(float value, float limit);
    static bool Is_Finite(float value);
};


/**
 * @brief MKSESC/VESC 轮向电机适配器
 *
 * 作用：
 * 把现有 Class_Motor_MKSESC 适配成 Class_Motor_Base，
 * 这样可以直接接入 Class_Swerve_Module。
 *
 * 推荐用途：
 * 轮向电机。
 *
 * 单位约定：
 * Set_Target_Speed(speed) 的 speed 单位是 m/s，
 * 适配器内部根据轮半径和减速比转换成电机机械角速度 rad/s。
 *
 * Set_Target_Current(current) 的 current 单位是 A，
 * 直接下发给 MKSESC 电流模式。
 */
class Class_Motor_MKSESC_Adapter : public Class_Motor_Base
{
public:
    struct Parameters
    {
        float Wheel_Radius_M = 0.05f;
        float Reduction_Ratio = 1.0f;

        /*
         * 是否在 Output() 里调用 TIM_Send_PeriodElapsedCallback。
         * 正常保持 true。
         */
        bool Output_Use_TIM_Send_Callback = true;
    };

public:
    Class_Motor_MKSESC_Adapter() = default;

    bool Init(
        Class_Motor_MKSESC& motor,
        const Parameters& parameters
    );

    void Init() override;

    void Set_Control_Mode(Enum_Motor_Control_Mode mode) override;

    void Set_Target_Current(float current) override;
    void Set_Target_Speed(float speed) override;
    void Set_Target_Position(float position) override;

    void Set_Feedback_Current(float current) override;
    void Set_Feedback_Speed(float speed) override;
    void Set_Feedback_Position(float position) override;

    float Get_Current() const override;
    float Get_Speed() const override;
    float Get_Position() const override;

    float Get_Target_Current() const;
    float Get_Target_Speed() const;
    float Get_Target_Position() const;

    bool Is_Initialized() const;

    void Update_Feedback() override;
    void Calculate() override;
    void Output() override;

private:
    Class_Motor_MKSESC* Motor = nullptr;
    Parameters Param;

    Enum_Motor_Control_Mode Control_Mode = MOTOR_CONTROL_MODE_DISABLE;

    float Target_Current = 0.0f;   // A
    float Target_Speed = 0.0f;     // m/s
    float Target_Position = 0.0f;  // wheel rad

    float Feedback_Current = 0.0f;   // A
    float Feedback_Speed = 0.0f;     // m/s
    float Feedback_Position = 0.0f;  // wheel rad

    bool Initialized = false;

private:
    float Speed_Mps_To_Motor_Radps(float speed_mps) const;
    float Motor_Radps_To_Speed_Mps(float motor_radps) const;
    float Wheel_Rad_To_Motor_Rad(float wheel_rad) const;
    float Motor_Rad_To_Wheel_Rad(float motor_rad) const;

    static bool Is_Finite(float value);
};

#endif
