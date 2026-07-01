#include "dvc_swerve_module.h"

namespace
{
    constexpr float kPi = 3.14159265358979323846f;
    constexpr float kHalfPi = kPi / 2.0f;
    constexpr float kTwoPi = 2.0f * kPi;
}

bool Class_Swerve_Module::Init(
    Class_Motor_Base& steer_motor,
    Class_Motor_Base& drive_motor,
    const Parameters& parameters
)
{
    if (!Check_Parameters(parameters))
    {
        Initialized = false;
        return false;
    }

    Steer_Motor = &steer_motor;
    Drive_Motor = &drive_motor;

    Param = parameters;

    Steer_Encoder.Init(
        Param.Steer_Zero_Offset_Rad,
        Param.Steer_Encoder_Reverse
    );

    Module_Target = {};
    Current_Mode = Mode::Speed;

    Initialized = true;

    return true;
}

bool Class_Swerve_Module::Check_Parameters(const Parameters& parameters) const
{
    if (!Is_Finite(parameters.Force_To_Current) ||
        !Is_Finite(parameters.Speed_Deadband) ||
        !Is_Finite(parameters.Force_Deadband) ||
        !Is_Finite(parameters.Max_Speed_Mps) ||
        !Is_Finite(parameters.Max_Force_N) ||
        !Is_Finite(parameters.Max_Current_A) ||
        !Is_Finite(parameters.Steer_Zero_Offset_Rad))
    {
        return false;
    }

    /*
     * Force_To_Current 必须为正。
     * 方向反的问题不建议通过负比例系数解决。
     */
    if (parameters.Force_To_Current <= 0.0f)
    {
        return false;
    }

    if (parameters.Speed_Deadband < 0.0f ||
        parameters.Force_Deadband < 0.0f)
    {
        return false;
    }

    /*
     * Max_* <= 0 表示不启用限幅，所以这里不判定为非法。
     */

    return true;
}

bool Class_Swerve_Module::Is_Initialized() const
{
    return Initialized;
}

void Class_Swerve_Module::Update_Encoder(uint16_t encoder_raw)
{
    if (!Initialized)
    {
        return;
    }

    Steer_Encoder.Update(encoder_raw);
}

bool Class_Swerve_Module::Set_Target_Speed_Angle(
    float speed_mps,
    float angle_rad
)
{
    if (!Is_Finite(speed_mps) || !Is_Finite(angle_rad))
    {
        Clear_Drive_Target();
        return false;
    }

    Current_Mode = Mode::Speed;

    Module_Target.Speed_Mps = Limit(speed_mps, Param.Max_Speed_Mps);
    Module_Target.Force_N = 0.0f;
    Module_Target.Current_A = 0.0f;
    Module_Target.Angle_Rad = Normalize_Angle(angle_rad);

    return true;
}

bool Class_Swerve_Module::Set_Target_Force_Angle(
    float force_n,
    float angle_rad
)
{
    if (!Is_Finite(force_n) || !Is_Finite(angle_rad))
    {
        Clear_Drive_Target();
        return false;
    }

    Current_Mode = Mode::Force;

    Module_Target.Speed_Mps = 0.0f;
    Module_Target.Force_N = Limit(force_n, Param.Max_Force_N);
    Module_Target.Current_A = 0.0f;
    Module_Target.Angle_Rad = Normalize_Angle(angle_rad);

    return true;
}

bool Class_Swerve_Module::Set_Target_Speed_Force_Angle(
    float speed_mps,
    float force_n,
    float angle_rad
)
{
    if (!Is_Finite(speed_mps) || !Is_Finite(force_n) || !Is_Finite(angle_rad))
    {
        Clear_Drive_Target();
        return false;
    }

    Current_Mode = Mode::Speed_Force;

    Module_Target.Speed_Mps = Limit(speed_mps, Param.Max_Speed_Mps);
    Module_Target.Force_N = Limit(force_n, Param.Max_Force_N);
    Module_Target.Current_A = 0.0f;
    Module_Target.Angle_Rad = Normalize_Angle(angle_rad);

    return true;
}

void Class_Swerve_Module::Calculate()
{
    if (!Initialized)
    {
        return;
    }

    /*
     * 先做舵向角度优化。
     * 可能会改变：
     * 1. Target Angle
     * 2. Target Speed
     * 3. Target Force
     */
    Optimize_Target();

    /*
     * 力控目标转换成电流目标。
     */
    Module_Target.Current_A =
        Module_Target.Force_N * Param.Force_To_Current;

    Module_Target.Current_A = Limit(
        Module_Target.Current_A,
        Param.Max_Current_A
    );

    Apply_Steer_Target();
    Apply_Drive_Target();
}

void Class_Swerve_Module::Apply_Steer_Target()
{
    if (Steer_Motor == nullptr)
    {
        return;
    }

    /*
     * 舵向电机始终工作在位置模式。
     */
    Steer_Motor->Set_Control_Mode(MOTOR_CONTROL_MODE_POSITION);

    /*
     * 舵向位置反馈使用外置绝对编码器。
     */
    Steer_Motor->Set_Feedback_Position(
        Steer_Encoder.Get_Angle_Rad()
    );

    Steer_Motor->Set_Target_Position(
        Module_Target.Angle_Rad
    );

    Steer_Motor->Calculate();
}

void Class_Swerve_Module::Apply_Drive_Target()
{
    if (Drive_Motor == nullptr)
    {
        return;
    }

    if (Current_Mode == Mode::Speed)
    {
        /*
         * 速度模式：
         * 这里的 Target_Speed 单位是 m/s。
         * 具体电机类需要在内部转换成 RPM / ERPM / 其他驱动器单位。
         */
        Drive_Motor->Set_Control_Mode(MOTOR_CONTROL_MODE_SPEED);

        Drive_Motor->Set_Target_Speed(
            Module_Target.Speed_Mps
        );
    }
    else
    {
        /*
         * Force 和 Speed_Force 模式：
         * 轮向电机跑电流环 / 力控。
         */
        Drive_Motor->Set_Control_Mode(MOTOR_CONTROL_MODE_CURRENT);

        Drive_Motor->Set_Target_Current(
            Module_Target.Current_A
        );
    }

    Drive_Motor->Calculate();
}

void Class_Swerve_Module::Optimize_Target()
{
    if (Is_In_Deadband())
    {
        Clear_Drive_Target();
        return;
    }

    const float current_angle = Steer_Encoder.Get_Angle_Rad();

    float error = Module_Target.Angle_Rad - current_angle;
    error = Normalize_Angle(error);

    /*
     * 舵向优化：
     *
     * 如果目标角度和当前角度误差超过 90°，
     * 说明直接转过去太远。
     *
     * 此时：
     * 1. 舵向目标角反向 180°
     * 2. 轮向速度取反
     * 3. 轮向力取反
     */
    if (error > kHalfPi || error < -kHalfPi)
    {
        Flip_Direction();
    }
}

bool Class_Swerve_Module::Is_In_Deadband() const
{
    return fabsf(Module_Target.Speed_Mps) < Param.Speed_Deadband &&
           fabsf(Module_Target.Force_N) < Param.Force_Deadband;
}

void Class_Swerve_Module::Clear_Drive_Target()
{
    Module_Target.Speed_Mps = 0.0f;
    Module_Target.Force_N = 0.0f;
    Module_Target.Current_A = 0.0f;
}

void Class_Swerve_Module::Flip_Direction()
{
    Module_Target.Angle_Rad =
        Normalize_Angle(Module_Target.Angle_Rad + kPi);

    Module_Target.Speed_Mps = -Module_Target.Speed_Mps;
    Module_Target.Force_N = -Module_Target.Force_N;
}

void Class_Swerve_Module::Output()
{
    if (!Initialized)
    {
        return;
    }

    if (Steer_Motor != nullptr)
    {
        Steer_Motor->Output();
    }

    if (Drive_Motor != nullptr)
    {
        Drive_Motor->Output();
    }
}

void Class_Swerve_Module::Update()
{
    Calculate();
    Output();
}

void Class_Swerve_Module::Stop()
{
    Clear_Drive_Target();

    if (!Initialized)
    {
        return;
    }

    if (Drive_Motor != nullptr)
    {
        Drive_Motor->Set_Control_Mode(MOTOR_CONTROL_MODE_DISABLE);
        Drive_Motor->Set_Target_Speed(0.0f);
        Drive_Motor->Set_Target_Current(0.0f);
        Drive_Motor->Calculate();
    }

    /*
     * Stop 时舵向保持当前目标角，不强制回零。
     */
    if (Steer_Motor != nullptr)
    {
        Steer_Motor->Set_Control_Mode(MOTOR_CONTROL_MODE_POSITION);

        Steer_Motor->Set_Feedback_Position(
            Steer_Encoder.Get_Angle_Rad()
        );

        Steer_Motor->Set_Target_Position(
            Module_Target.Angle_Rad
        );

        Steer_Motor->Calculate();
    }
}

void Class_Swerve_Module::Stop_And_Output()
{
    Stop();
    Output();
}

Class_Swerve_Module::Mode Class_Swerve_Module::Get_Mode() const
{
    return Current_Mode;
}

float Class_Swerve_Module::Get_Current_Angle() const
{
    return Steer_Encoder.Get_Angle_Rad();
}

float Class_Swerve_Module::Get_Target_Angle() const
{
    return Module_Target.Angle_Rad;
}

float Class_Swerve_Module::Get_Target_Speed() const
{
    return Module_Target.Speed_Mps;
}

float Class_Swerve_Module::Get_Target_Force() const
{
    return Module_Target.Force_N;
}

float Class_Swerve_Module::Get_Target_Current() const
{
    return Module_Target.Current_A;
}

const Class_Swerve_Module::Target& Class_Swerve_Module::Get_Target() const
{
    return Module_Target;
}

float Class_Swerve_Module::Normalize_Angle(float angle)
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

float Class_Swerve_Module::Limit(float value, float limit)
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

bool Class_Swerve_Module::Is_Finite(float value)
{
    return isfinite(value);
}
