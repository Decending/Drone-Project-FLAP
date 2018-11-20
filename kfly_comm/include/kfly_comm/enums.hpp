//          Copyright Emil Fresk 2016 - 2017
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#ifndef _KFLY_ENUMS_H
#define _KFLY_ENUMS_H

#include <cstdint>

namespace kfly_comm
{
namespace enums
{

/* @brief The available flight modes via computer control. */
enum class FlightMode : uint8_t
{
    /* @brief Control each motor individually. */
    MOTOR_DIRECT_MODE = 1,

    /* @brief Control each motor through the mixing matrix. */
    MOTOR_INDIRECT_MODE = 2,

    /* @brief Control via rate references. */
    RATE_MODE = 3,

    /* @brief Control via Euler references (not implemented). */
    ATTITUDE_EULER_MODE = 4,

    /* @brief Control via attitude references. */
    ATTITUDE_MODE = 5
};

/* @brief Port selector, select from the USB or the UART ports.  */
enum class Ports : uint8_t
{
    /* @brief USB port. */
    PORT_USB = 0,

    /* @brief Aux 1 port. */
    PORT_AUX1 = 1,

    /* @brief Aux 2 port. */
    PORT_AUX2 = 2,

    /* @brief Aux 3 port. */
    PORT_AUX3 = 3,

    /* @brief Same as the message came on. */
    PORT_SAME = 0xff
};

/* @brief   Input capture channel role selector. */
enum class RCInput_Role : uint8_t
{

    /* @brief Throttle role selector. */
    ROLE_THROTTLE = 0,

    /* @brief Pitch role selector. */
    ROLE_PITCH = 1,

    /* @brief Roll role selector. */
    ROLE_ROLL = 2,

    /* @brief Yaw role selector. */
    ROLE_YAW = 3,

    /* @brief Aux 1 role selector. */
    ROLE_AUX1 = 4,

    /* @brief Aux 2 role selector.  */
    ROLE_AUX2 = 5,

    /*  @brief Aux 3 role selector. */
    ROLE_AUX3 = 6,

    /* @brief Role of a non-latching switch to arm the flight controller. */
    ROLE_ARM_NONLATCH = 7,

    /* @brief Role for a latching switch to switch to remote serial control. */
    ROLE_ENABLE_SERIAL_CONTROL = 8,

    /* @brief Aux 1 role selector. */
    ROLE_FLIGHTMODE = 9,

    /* @brief The number of roles - 1. Always have at the end! */
    ROLE_MAX = 10,

    /* @brief Unused role selector. */
    ROLE_OFF = 0xff,
};

/* @brief Input capture channel type. */
enum class RCInput_Type : uint8_t
{
    /* @brief Analog type input. */
    TYPE_ANALOG = 1,

    /* @brief 3-state type input. */
    TYPE_3_STATE = 2,

    /* @brief On/off type input. */
    TYPE_ON_OFF = 3
};

/* @brief Input capture type selector. */
enum class RCInput_Mode : uint8_t
{
    /** @brief   No mode. */
    MODE_NONE = 0,

    /** @brief   CPPM input: all channels on one input line. */
    MODE_CPPM_INPUT = 1,

    /** @brief   SBUS input: serial input */
    MODE_SBUS_INPUT = 2
};

/* @brief Input switch states. */
enum class RCInput_Switch_Position : uint8_t
{
    /* @brief   Switch error, not a switch. */
    RCINPUT_SWITCH_UNDEFINED = 0,

    /* @brief   Switch error, not a switch. */
    RCINPUT_SWITCH_NOT_SWITCH = 1,

    /* @brief   Switch at bottom position. */
    RCINPUT_SWITCH_POSITION_BOTTOM = 2,

    /* @brief   Switch at center position. */
    RCINPUT_SWITCH_POSITION_CENTER = 3,

    /* @brief   Switch at top position. */
    RCINPUT_SWITCH_POSITION_TOP = 4
};

/**
 * @brief Enum for selecting the period of the output PPM.
 */
enum class RCOutput_Mode : uint8_t
{
    RCOUTPUT_MODE_50HZ_PWM = 0,
    RCOUTPUT_MODE_400HZ_PWM,
    RCOUTPUT_MODE_ONESHOT125,
    RCOUTPUT_MODE_ONESHOT42,
    RCOUTPUT_MODE_MULTISHOT,
    RCOUTPUT_MODE_DSHOT150,
    RCOUTPUT_MODE_DSHOT300,
    RCOUTPUT_MODE_DSHOT600,
    RCOUTPUT_MODE_DSHOT1200,
};

/**
 * @brief   Possible stick direction for arming the controllers.
 */
enum class Arming_Stick_Direction : uint8_t
{
    /* @brief   Arm direction not yet set. */
    STICK_NONE = 0,

    /* @brief Arm at pitch at min. */
    STICK_PITCH_MIN,

    /* @brief Arm at pitch at max. */
    STICK_PITCH_MAX,

    /* @brief Arm at roll at min. */
    STICK_ROLL_MIN,

    /* @brief Arm at roll at max. */
    STICK_ROLL_MAX,

    /* @brief Arm at yaw at min. */
    STICK_YAW_MIN,

    /* @brief Arm at yaw at max. */
    STICK_YAW_MAX,

    /* @brief Arm with the non-latching switch. */
    NON_LATCHING_SWITCH
};

/**
 * @brief Supported biquad types.
 */
enum class BiquadType : uint8_t
{
    LPF,
    HPF,
    NOTCH
};

/**
 * @brief Supported biquad modes.
 */
enum class BiquadMode : uint8_t
{
    BIQUAD, // 2nd order filter
    PT1     // first order filter
};

} // namespace enums
} // namespace kfly_comm

#endif
