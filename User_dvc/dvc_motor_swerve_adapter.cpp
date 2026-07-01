#include "dvc_motor_swerve_adapter.h"

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kTwoPi = 2.0f * kPi;
}

/* DJI C610 Adapter -------------------------------------------------------*/

bool Class_Motor_DJI_C610_Adapter::Init(
    Class_Motor_DJI_C610& motor,
    const Parameters& parameters
)
{
    if (!Is_Finite(parameters.Position_Kp) ||
        !Is_Finite(parameters.Position_Ki) ||
        !Is_Finite(parameters.Position_Kd) ||
        !Is_Finite(parameters.Position_Integral_Limit) ||
        !Is_Finite(parameters.Position_Output_Limit))
    {
        Initialized = false;
        return false;
    }

    Motor = &motor;
    Param = parameters;

    Control_Mode = MOTOR_CONTROL_MODE_DISABLE;

    Target_Current = 0.0f;
    Target_Speed = 0.0f;
    Target_Position = 0.0f;

    Feedback_Current = 0.0f;
    Feedback_Speed = 0.0f;
    Feedback_Position = 0.0f;

    Clear_Position_PID();

    Initialized = true;

    return true;
}

void Class_Motor_DJI_C610_Adapter::Init()
{
    /*
     * 兼容 Class_Motor_Base 的无参 Init。
     * 真正绑定底层电机请调用 Init(Class_Motor_DJI_GM6020&, Parameters)。
     */
}

void Class_Motor_DJI_C610_Adapter::Set_Control_Mode(Enum_Motor_Control_Mode mode)
{
    Control_Mode = mode;
}

void Class_Motor_DJI_C610_Adapter::Set_Target_Current(float current)
{
    if (Is_Finite(current))
    {
        Target_Current = current;
    }
}

void Class_Motor_DJI_C610_Adapter::Set_Target_Speed(float speed)
{
    if (Is_Finite(speed))
    {
        /*
         * C610 舵向适配器中 speed 单位为 rad/s。
         */
        Target_Speed = speed;
    }
}

void Class_Motor_DJI_C610_Adapter::Set_Target_Position(float position)
{
    if (Is_Finite(position))
    {
        Target_Position = Normalize_Angle(position);
    }
}

void Class_Motor_DJI_C610_Adapter::Set_Feedback_Current(float current)
{
    if (Is_Finite(current))
    {
        Feedback_Current = current;
    }
}

void Class_Motor_DJI_C610_Adapter::Set_Feedback_Speed(float speed)
{
    if (Is_Finite(speed))
    {
        Feedback_Speed = speed;
    }
}

void Class_Motor_DJI_C610_Adapter::Set_Feedback_Position(float position)
{
    if (Is_Finite(position))
    {
        Feedback_Position = Normalize_Angle(position);
    }
}

float Class_Motor_DJI_C610_Adapter::Get_Current() const
{
    return Feedback_Current;
}

float Class_Motor_DJI_C610_Adapter::Get_Speed() const
{
    return Feedback_Speed;
}

float Class_Motor_DJI_C610_Adapter::Get_Position() const
{
    return Feedback_Position;
}

float Class_Motor_DJI_C610_Adapter::Get_Target_Current() const
{
    return Target_Current;
}

float Class_Motor_DJI_C610_Adapter::Get_Target_Speed() const
{
    return Target_Speed;
}

float Class_Motor_DJI_C610_Adapter::Get_Target_Position() const
{
    return Target_Position;
}

bool Class_Motor_DJI_C610_Adapter::Is_Initialized() const
{
    return Initialized;
}

void Class_Motor_DJI_C610_Adapter::Update_Feedback()
{
    if (!Initialized || Motor == nullptr)
    {
        return;
    }

    Feedback_Current = Motor->Get_Now_Current();
    Feedback_Speed = Motor->Get_Now_Omega();

    /*
     * 舵轮推荐使用外置绝对编码器位置反馈。
     * 如果 Use_External_Position_Feedback = true，则不覆盖外部 Set_Feedback_Position() 写入的位置。
     */
    if (!Param.Use_External_Position_Feedback)
    {
        Feedback_Position = Normalize_Angle(Motor->Get_Now_Angle());
    }
}

void Class_Motor_DJI_C610_Adapter::Calculate()
{
    if (!Initialized || Motor == nullptr)
    {
        return;
    }

    Update_Feedback();

    switch (Control_Mode)
    {
    case MOTOR_CONTROL_MODE_CURRENT:
    {
        Clear_Position_PID();
        Motor->Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
        Motor->Set_Target_Current(Target_Current);
        break;
    }

    case MOTOR_CONTROL_MODE_SPEED:
    {
        Clear_Position_PID();
        Motor->Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor->Set_Target_Omega(Target_Speed);
        break;
    }

    case MOTOR_CONTROL_MODE_POSITION:
    {
        const float target_speed = Calculate_Position_PID(
            Target_Position,
            Feedback_Position
        );

        Motor->Set_Control_Method(Motor_DJI_Control_Method_OMEGA);
        Motor->Set_Target_Omega(target_speed);
        break;
    }

    case MOTOR_CONTROL_MODE_DISABLE:
    default:
    {
        Clear_Position_PID();
        Motor->Set_Control_Method(Motor_DJI_Control_Method_CURRENT);
        Motor->Set_Target_Current(0.0f);
        break;
    }
    }
}

void Class_Motor_DJI_C610_Adapter::Output()
{
    if (!Initialized || Motor == nullptr)
    {
        return;
    }

    /*
     * DJI 原始类中 TIM_Calculate_PeriodElapsedCallback() 会完成 PID_Calculate()
     * 并写入 CAN 发送缓冲区。
     */
    if (Param.Use_Power_Limit_After_Calculate)
    {
    //Motor->TIM_Power_Limit_After_Calculate_PeriodElapsedCallback();
    }
    else
    {
        Motor->TIM_Calculate_PeriodElapsedCallback();
    }
}

float Class_Motor_DJI_C610_Adapter::Calculate_Position_PID(
    float target,
    float feedback
)
{
    Position_Error = Normalize_Angle(target - feedback);

    Position_Integral += Position_Error;
    Position_Integral = Limit(
        Position_Integral,
        Param.Position_Integral_Limit
    );

    const float derivative = Position_Error - Position_Last_Error;

    float output =
        Param.Position_Kp * Position_Error +
        Param.Position_Ki * Position_Integral +
        Param.Position_Kd * derivative;

    output = Limit(output, Param.Position_Output_Limit);

    Position_Last_Error = Position_Error;

    return output;
}

void Class_Motor_DJI_C610_Adapter::Clear_Position_PID()
{
    Position_Error = 0.0f;
    Position_Last_Error = 0.0f;
    Position_Integral = 0.0f;
}

float Class_Motor_DJI_C610_Adapter::Normalize_Angle(float angle)
{
    if (!Is_Finite(angle))
    {
        return 0.0f;
    }

    angle = fmodf(angle, kTwoPi);

    if (angle > kPi)
    {
        angle -= kTwoPi;
    }
    else if (angle < -kPi)
    {
        angle += kTwoPi;
    }

    return angle;
}

float Class_Motor_DJI_C610_Adapter::Limit(float value, float limit)
{
    if (limit <= 0.0f)
    {
        return value;
    }

    if (value > limit)
    {
        return limit;
    }

    if (value < -limit)
    {
        return -limit;
    }

    return value;
}

bool Class_Motor_DJI_C610_Adapter::Is_Finite(float value)
{
    return isfinite(value);
}


/* MKSESC Adapter -----------------------------------------------------------*/

bool Class_Motor_MKSESC_Adapter::Init(
    Class_Motor_MKSESC& motor,
    const Parameters& parameters
)
{
    if (!Is_Finite(parameters.Wheel_Radius_M) ||
        !Is_Finite(parameters.Reduction_Ratio) ||
        parameters.Wheel_Radius_M <= 0.0f ||
        parameters.Reduction_Ratio <= 0.0f)
    {
        Initialized = false;
        return false;
    }

    Motor = &motor;
    Param = parameters;

    Control_Mode = MOTOR_CONTROL_MODE_DISABLE;

    Target_Current = 0.0f;
    Target_Speed = 0.0f;
    Target_Position = 0.0f;

    Feedback_Current = 0.0f;
    Feedback_Speed = 0.0f;
    Feedback_Position = 0.0f;

    Initialized = true;

    return true;
}

void Class_Motor_MKSESC_Adapter::Init()
{
    /*
     * 兼容 Class_Motor_Base 的无参 Init。
     * 真正绑定底层电机请调用 Init(Class_Motor_MKSESC&, Parameters)。
     */
}

void Class_Motor_MKSESC_Adapter::Set_Control_Mode(Enum_Motor_Control_Mode mode)
{
    Control_Mode = mode;
}

void Class_Motor_MKSESC_Adapter::Set_Target_Current(float current)
{
    if (Is_Finite(current))
    {
        Target_Current = current;
    }
}

void Class_Motor_MKSESC_Adapter::Set_Target_Speed(float speed)
{
    if (Is_Finite(speed))
    {
        /*
         * 对外单位：m/s。
         */
        Target_Speed = speed;
    }
}

void Class_Motor_MKSESC_Adapter::Set_Target_Position(float position)
{
    if (Is_Finite(position))
    {
        /*
         * 对外单位：轮端 rad。
         */
        Target_Position = position;
    }
}

void Class_Motor_MKSESC_Adapter::Set_Feedback_Current(float current)
{
    if (Is_Finite(current))
    {
        Feedback_Current = current;
    }
}

void Class_Motor_MKSESC_Adapter::Set_Feedback_Speed(float speed)
{
    if (Is_Finite(speed))
    {
        Feedback_Speed = speed;
    }
}

void Class_Motor_MKSESC_Adapter::Set_Feedback_Position(float position)
{
    if (Is_Finite(position))
    {
        Feedback_Position = position;
    }
}

float Class_Motor_MKSESC_Adapter::Get_Current() const
{
    return Feedback_Current;
}

float Class_Motor_MKSESC_Adapter::Get_Speed() const
{
    return Feedback_Speed;
}

float Class_Motor_MKSESC_Adapter::Get_Position() const
{
    return Feedback_Position;
}

float Class_Motor_MKSESC_Adapter::Get_Target_Current() const
{
    return Target_Current;
}

float Class_Motor_MKSESC_Adapter::Get_Target_Speed() const
{
    return Target_Speed;
}

float Class_Motor_MKSESC_Adapter::Get_Target_Position() const
{
    return Target_Position;
}

bool Class_Motor_MKSESC_Adapter::Is_Initialized() const
{
    return Initialized;
}

void Class_Motor_MKSESC_Adapter::Update_Feedback()
{
    if (!Initialized || Motor == nullptr)
    {
        return;
    }

    Feedback_Current = Motor->Get_Now_Current();
    Feedback_Speed = Motor_Radps_To_Speed_Mps(Motor->Get_Now_Omega());
    Feedback_Position = Motor_Rad_To_Wheel_Rad(Motor->Get_Now_Angle());
}

void Class_Motor_MKSESC_Adapter::Calculate()
{
    if (!Initialized || Motor == nullptr)
    {
        return;
    }

    Update_Feedback();

    switch (Control_Mode)
    {
    case MOTOR_CONTROL_MODE_CURRENT:
    {
        Motor->Set_Control_Method(Motor_MKSESC_Control_Method_Current);
        Motor->Set_Control_Current(Target_Current);
        break;
    }

    case MOTOR_CONTROL_MODE_SPEED:
    {
        Motor->Set_Control_Method(Motor_MKSESC_Control_Method_Omega);
        Motor->Set_Control_Omega(
            Speed_Mps_To_Motor_Radps(Target_Speed)
        );
        break;
    }

    case MOTOR_CONTROL_MODE_POSITION:
    {
        Motor->Set_Control_Method(Motor_MKSESC_Control_Method_Angle);
        Motor->Set_Control_Angle(
            Wheel_Rad_To_Motor_Rad(Target_Position)
        );
        break;
    }

    case MOTOR_CONTROL_MODE_DISABLE:
    default:
    {
        Motor->Set_Control_Method(Motor_MKSESC_Control_Method_Current);
        Motor->Set_Control_Current(0.0f);
        break;
    }
    }
}

void Class_Motor_MKSESC_Adapter::Output()
{
    if (!Initialized || Motor == nullptr)
    {
        return;
    }

    if (Param.Output_Use_TIM_Send_Callback)
    {
        Motor->TIM_Send_PeriodElapsedCallback();
    }
}

float Class_Motor_MKSESC_Adapter::Speed_Mps_To_Motor_Radps(float speed_mps) const
{
    return speed_mps / Param.Wheel_Radius_M * Param.Reduction_Ratio;
}

float Class_Motor_MKSESC_Adapter::Motor_Radps_To_Speed_Mps(float motor_radps) const
{
    return motor_radps / Param.Reduction_Ratio * Param.Wheel_Radius_M;
}

float Class_Motor_MKSESC_Adapter::Wheel_Rad_To_Motor_Rad(float wheel_rad) const
{
    return wheel_rad * Param.Reduction_Ratio;
}

float Class_Motor_MKSESC_Adapter::Motor_Rad_To_Wheel_Rad(float motor_rad) const
{
    return motor_rad / Param.Reduction_Ratio;
}

bool Class_Motor_MKSESC_Adapter::Is_Finite(float value)
{
    return isfinite(value);
}
