#ifndef DVC_SWERVE_MODULE_H
#define DVC_SWERVE_MODULE_H

#include <stdint.h>
#include <math.h>

#include "dvc_motor_base.h"
#include "dvc_swerve_steer_encoder.h"

/**
 * @brief 单个舵轮模块
 *
 * 职责：
 * 1. 接收轮组下发的目标舵角、目标轮速、目标牵引力
 * 2. 根据当前绝对编码器角度做舵向优化
 * 3. 控制舵向电机进入位置模式
 * 4. 控制轮向电机进入速度模式或电流模式
 *
 * 注意：
 * 这个类不负责底盘运动学解算，也不直接发送 CAN。
 *
 * 初始化方式：
 * 1. 嵌入式推荐：默认构造 + Init()
 * 2. 局部/测试可选：构造函数直接初始化
 */
class Class_Swerve_Module
{
public:
    enum class Mode : uint8_t
    {
        Speed = 0,       // 轮向速度环
        Force,           // 轮向电流/力控
        Speed_Force,     // 舵角由速度决定，轮向由力控决定
    };

    struct Parameters
    {
        /*
         * 牵引力 N -> 电机电流 A 的简化比例。
         *
         * current = force * Force_To_Current
         *
         * 更严谨的公式：
         * current = force * wheel_radius / reduction_ratio / kt
         *
         * 注意：
         * Force_To_Current 必须大于 0。
         * 方向反的问题应通过电机方向、VESC 方向或轮组 Drive_Direction 处理。
         */
        float Force_To_Current = 0.2f;

        /*
         * 小于死区时认为目标为 0，避免停止或低速时舵轮乱优化。
         */
        float Speed_Deadband = 0.03f; // m/s
        float Force_Deadband = 0.5f;  // N

        /*
         * 目标限幅。
         * <= 0 表示不启用对应限幅。
         */
        float Max_Speed_Mps = 0.0f;   // m/s
        float Max_Force_N = 0.0f;     // N
        float Max_Current_A = 0.0f;   // A

        /*
         * 舵向绝对编码器参数。
         */
        float Steer_Zero_Offset_Rad = 0.0f;
        bool Steer_Encoder_Reverse = false;
    };

    struct Target
    {
        float Angle_Rad = 0.0f;  // 舵向目标角，rad
        float Speed_Mps = 0.0f;  // 轮向目标速度，m/s
        float Force_N = 0.0f;    // 轮向目标牵引力，N
        float Current_A = 0.0f;  // 轮向目标电流，A
    };

public:
    /*
     * 默认构造：
     * 适合全局对象、静态对象、数组对象。
     * 使用前必须调用 Init()。
     */
    Class_Swerve_Module() = default;

    /*
     * 直接构造：
     * 适合局部对象、测试对象或依赖初始化顺序明确的场景。
     */
    explicit Class_Swerve_Module(
        Class_Motor_Base& steer_motor,
        Class_Motor_Base& drive_motor,
        const Parameters& parameters
    )
    {
        Init(steer_motor, drive_motor, parameters);
    }

    /*
     * 初始化函数：
     * 嵌入式主路径。建议在外设、电机、编码器相关对象完成初始化后调用。
     *
     * 返回值：
     * true  初始化成功
     * false 参数非法，初始化失败
     */
    bool Init(
        Class_Motor_Base& steer_motor,
        Class_Motor_Base& drive_motor,
        const Parameters& parameters
    );

    bool Is_Initialized() const;

    /*
     * 更新舵向绝对编码器原始值。
     * 建议在每个控制周期最先调用。
     */
    void Update_Encoder(uint16_t encoder_raw);

    /*
     * 三种目标设置接口。
     *
     * speed_mps: 轮向线速度，单位 m/s
     * force_n:  轮向牵引力，单位 N
     * angle_rad: 舵向目标角，单位 rad
     *
     * 返回值：
     * true  表示目标有效并已设置
     * false 表示目标存在 NaN/Inf 等非法值，模块会清除驱动目标
     */
    bool Set_Target_Speed_Angle(float speed_mps, float angle_rad);
    bool Set_Target_Force_Angle(float force_n, float angle_rad);
    bool Set_Target_Speed_Force_Angle(
        float speed_mps,
        float force_n,
        float angle_rad
    );

    /*
     * 控制周期调用。
     */
    void Calculate();
    void Output();

    /*
     * 便捷接口：等价于 Calculate() + Output()。
     */
    void Update();

    /*
     * Stop 只修改目标并计算，不强制发送。
     * 如果需要立即下发，请调用 Stop_And_Output()。
     */
    void Stop();
    void Stop_And_Output();

    /*
     * 状态读取接口。
     */
    Mode Get_Mode() const;

    float Get_Current_Angle() const;
    float Get_Target_Angle() const;
    float Get_Target_Speed() const;
    float Get_Target_Force() const;
    float Get_Target_Current() const;

    const Target& Get_Target() const;

private:
    Class_Motor_Base* Steer_Motor = nullptr;
    Class_Motor_Base* Drive_Motor = nullptr;

    Class_Swerve_Steer_Encoder Steer_Encoder;

    Parameters Param;
    Target Module_Target;

    Mode Current_Mode = Mode::Speed;

    bool Initialized = false;

private:
    bool Check_Parameters(const Parameters& parameters) const;

    void Optimize_Target();

    bool Is_In_Deadband() const;
    void Clear_Drive_Target();
    void Flip_Direction();

    void Apply_Steer_Target();
    void Apply_Drive_Target();

    static float Normalize_Angle(float angle);
    static float Limit(float value, float limit);
    static bool Is_Finite(float value);
};

#endif
