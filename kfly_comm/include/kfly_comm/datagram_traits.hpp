//          Copyright Emil Fresk 2016 - 2017
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <type_traits>
#include <kfly_comm/datagrams.hpp>

namespace kfly_comm
{
template < typename... Ts >
struct always_false : std::integral_constant< bool, false >
{
};

/*********************************
 * command_traits
 ********************************/

namespace command_traits
{

/**
 * @brief   Type traits to extract the command from a datagram, the command is
 *          available in ::value.
 *
 * @tparam  Datagram    The datagram to get the command for.
 */
template < typename Datagram >
struct get_packet_command
{
  static_assert(
      always_false< Datagram >::value,
      "This datagram cannot be sent to KFly, it's only for receiving.");
};

template <>
struct get_packet_command< datagrams::ManageSubscription >
    : std::integral_constant< commands, commands::ManageSubscriptions >
{
};

template <>
struct get_packet_command< datagrams::SystemStatus >
    : std::integral_constant< commands, commands::GetSystemStatus >
{
};

template <>
struct get_packet_command< datagrams::SystemStrings >
    : std::integral_constant< commands, commands::GetSystemStrings >
{
};

template <>
struct get_packet_command< datagrams::SetDeviceStrings >
    : std::integral_constant< commands, commands::SetDeviceStrings >
{
};

template <>
struct get_packet_command< datagrams::MotorOverride >
    : std::integral_constant< commands, commands::MotorOverride >
{
};

template <>
struct get_packet_command< datagrams::ControllerLimits >
    : std::integral_constant< commands, commands::SetControllerLimits >
{
};

template <>
struct get_packet_command< datagrams::ArmSettings >
    : std::integral_constant< commands, commands::SetArmSettings >
{
};

template <>
struct get_packet_command< datagrams::RateControllerData >
    : std::integral_constant< commands, commands::SetRateControllerData >
{
};

template <>
struct get_packet_command< datagrams::AttitudeControllerData >
    : std::integral_constant< commands, commands::SetAttitudeControllerData >
{
};

template <>
struct get_packet_command< datagrams::ChannelMix >
    : std::integral_constant< commands, commands::SetChannelMix >
{
};

template <>
struct get_packet_command< datagrams::RCInputSettings >
    : std::integral_constant< commands, commands::SetRCInputSettings >
{
};

template <>
struct get_packet_command< datagrams::RCOutputSettings >
    : std::integral_constant< commands, commands::SetRCOutputSettings >
{
};

template <>
struct get_packet_command< datagrams::IMUCalibration >
    : std::integral_constant< commands, commands::SetIMUCalibration >
{
};

template <>
struct get_packet_command< datagrams::ControlFilterSettings >
    : std::integral_constant< commands, commands::SetControlFilters >
{
};

template <>
struct get_packet_command< datagrams::ComputerControlReference >
    : std::integral_constant< commands, commands::ComputerControlReference >
{
};

template <>
struct get_packet_command< datagrams::MotionCaptureFrame >
    : std::integral_constant< commands, commands::MotionCaptureMeasurement >
{
};

} /* END command_traits*/
}
