#ifndef DVC_MOTOR_BASE_H
#define DVC_MOTOR_BASE_H

#include <stdint.h>

/**
 * @brief 通用电机控制模式
 *
 * 这个枚举只描述上层希望电机处于什么控制模式，
 * 具体如何转换成 CAN 指令、PWM、ERPM、电流输出，由子类/适配器负责。
 */
enum Enum_Motor_Control_Mode
{
    MOTOR_CONTROL_MODE_DISABLE = 0,  // 失能 / 零输出
    MOTOR_CONTROL_MODE_CURRENT,      // 电流模式，单位 A
    MOTOR_CONTROL_MODE_SPEED,        // 速度模式，单位由子类约定，舵轮工程中建议统一为 rad/s 或 m/s
    MOTOR_CONTROL_MODE_POSITION,     // 位置模式，单位 rad
    MOTOR_CONTROL_MODE_MIT,          // 预留 MIT 模式
};

/**
 * @brief 通用电机抽象基类
 *
 * 设计原则：
 * 1. 基类只定义统一接口，不关心具体电机协议。
 * 2. DJI、MKSESC、VESC、达妙等具体电机通过继承或适配器实现这些接口。
 * 3. 上层模块如 Class_Swerve_Module 只依赖 Class_Motor_Base。
 *
 * 单位建议：
 * - Current: A
 * - Position: rad
 * - Speed:
 *   - 舵向电机建议 rad/s
 *   - 轮向电机在舵轮模块中建议使用 m/s，由轮向适配器内部转换成 ERPM/rad/s
 */
class Class_Motor_Base
{
public:
    virtual ~Class_Motor_Base() {}

    /**
     * @brief 无参初始化接口
     *
     * 对于适配器类，真正的绑定初始化通常会额外提供带参数 Init()。
     * 这个无参 Init() 主要用于统一接口。
     */
    virtual void Init() = 0;

    /**
     * @brief 设置电机控制模式
     */
    virtual void Set_Control_Mode(Enum_Motor_Control_Mode mode) = 0;

    /**
     * @brief 设置目标电流，单位 A
     */
    virtual void Set_Target_Current(float current) = 0;

    /**
     * @brief 设置目标速度
     *
     * 对于舵向电机，建议单位 rad/s。
     * 对于轮向电机，在舵轮系统中建议单位 m/s，
     * 由具体轮向电机适配器换算为电机自身单位。
     */
    virtual void Set_Target_Speed(float speed) = 0;

    /**
     * @brief 设置目标位置，单位 rad
     */
    virtual void Set_Target_Position(float position) = 0;

    /**
     * @brief 写入外部反馈电流，单位 A
     *
     * 某些电机反馈来自自身 CAN；某些闭环反馈来自外部传感器。
     * 例如舵向电机位置反馈常来自绝对编码器。
     */
    virtual void Set_Feedback_Current(float current) = 0;

    /**
     * @brief 写入外部反馈速度
     */
    virtual void Set_Feedback_Speed(float speed) = 0;

    /**
     * @brief 写入外部反馈位置，单位 rad
     */
    virtual void Set_Feedback_Position(float position) = 0;

    /**
     * @brief 获取当前反馈电流，单位 A
     */
    virtual float Get_Current() const = 0;

    /**
     * @brief 获取当前反馈速度
     */
    virtual float Get_Speed() const = 0;

    /**
     * @brief 获取当前反馈位置，单位 rad
     */
    virtual float Get_Position() const = 0;

    /**
     * @brief 更新反馈
     *
     * 对于 CAN 电机，可以在这里从底层对象读取最新反馈；
     * 对于外部传感器反馈，可以由上层 Set_Feedback_* 写入。
     */
    virtual void Update_Feedback() = 0;

    /**
     * @brief 计算控制量
     *
     * 例如：
     * - DJI 舵向电机：位置外环 -> 速度目标
     * - MKSESC 轮向电机：m/s -> motor rad/s / ERPM
     */
    virtual void Calculate() = 0;

    /**
     * @brief 输出控制量
     *
     * 例如：
     * - 写入 CAN 发送缓冲区
     * - 直接发送 CAN 指令
     * - 输出 PWM
     */
    virtual void Output() = 0;
};

#endif
