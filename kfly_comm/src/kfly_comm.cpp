//          Copyright Emil Fresk 2016 - 2017
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#include "kfly_comm/kfly_comm.hpp"

namespace kfly_comm
{
/*********************************
 * Private members
 ********************************/

void codec::parse_packet(const std::vector< uint8_t > &payload)
{
  /* Check size */
  if (payload.size() < 4)
      return;

  /* Extract command. */
  const uint8_t cmd = payload[0];

  /* Extract size and expected size. */
  const unsigned int expected_size = (unsigned int)payload[1];
  const unsigned int length        = payload.size();

  /* Extract the CRC and remove it. */
  union {
    uint8_t b[2];
    uint16_t u16;
  } crc;

  crc.b[0] = payload[length - 2];
  crc.b[1] = payload[length - 1];

  /* Remove CRC. */
  std::vector< uint8_t > data(payload.begin(), payload.end() - 2);

  /* Calculate CRC. */
  uint16_t crc_calc = CRC16_CCITT::generateCRC(data);

  /* Remove header. */
  data.erase(data.begin(), data.begin() + 2);

  /* Check sizes and CRC. */
  if (expected_size + 4 != length)
  {
    /* Size error. */
    return;
  }
  else if (crc_calc != crc.u16)
  {
    /* CRC error. */
    return;
  }
  else
  {
    /* Send payload to further processing. */
    try
    {
      transmit_datagram(cmd, data);
    }
    catch (const std::invalid_argument &e)
    {
      /* Whenever a wrong sized payload is parsed, this will run. */
    }
  }
}

void codec::transmit_datagram(const uint8_t cmd,
                              const std::vector< uint8_t > &payload)
{
  /* Extract the command. */
  const commands command = static_cast< commands >(cmd);

  /* Do appropriate operation for each command. */
  switch (command)
  {
    case commands::ACK:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::Ack >().get_datagram());
      break;

    case commands::Ping:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::Ping >().get_datagram());
      break;

    case commands::GetRunningMode:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::RunningMode >(payload)
              .get_datagram());
      break;

    case commands::GetSystemStrings:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::SystemStrings >(payload)
              .get_datagram());
      break;

    case commands::GetSystemStatus:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::SystemStatus >(payload)
              .get_datagram());
      break;

    case commands::GetControlSignals:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::ControlSignals >(payload)
              .get_datagram());
      break;

    case commands::GetControllerReferences:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::ControllerReferences >(payload)
              .get_datagram());
      break;

    case commands::GetControllerLimits:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::ControllerLimits >(payload)
              .get_datagram());
      break;

    case commands::GetArmSettings:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::ArmSettings >(payload)
              .get_datagram());
      break;

    case commands::GetRateControllerData:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::RateControllerData >(payload)
              .get_datagram());
      break;

    case commands::GetAttitudeControllerData:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::AttitudeControllerData >(payload)
              .get_datagram());
      break;

    case commands::GetChannelMix:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::ChannelMix >(payload)
              .get_datagram());
      break;

    case commands::GetRCInputSettings:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::RCInputSettings >(payload)
              .get_datagram());
      break;

    case commands::GetRCOutputSettings:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::RCOutputSettings >(payload)
              .get_datagram());
      break;

    case commands::GetRCValues:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::RCValues >(payload).get_datagram());
      break;

    case commands::GetIMUData:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::IMUData >(payload).get_datagram());
      break;

    case commands::GetRawIMUData:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::RawIMUData >(payload)
              .get_datagram());
      break;

    case commands::GetIMUCalibration:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::IMUCalibration >(payload)
              .get_datagram());
      break;

    case commands::GetEstimationAttitude:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::EstimationAttitude >(payload)
              .get_datagram());
      break;

    case commands::GetControlFilters:

      _callbacks.execute_callback(
          serializable_datagram< datagrams::ControlFilterSettings >(payload)
              .get_datagram());
      break;

    default:
      break;
  }
}

/*********************************
 * Public members
 ********************************/

codec::codec() : _parser()
{
  std::lock_guard< std::mutex > locker(_parser_lock);

  /* Register the KFly packet parser to the parser output. */
  _parser.registerCallback([&](const std::vector< uint8_t > &payload) {
    this->parse_packet(payload);
  });
}

codec::~codec()
{
}

void codec::parse(const uint8_t data)
{
  std::lock_guard< std::mutex > locker(_parser_lock);

  _parser.parse(data);
}

void codec::parse(const std::vector< uint8_t > &payload)
{
  std::lock_guard< std::mutex > locker(_parser_lock);

  _parser.parse(payload);
}

std::vector< uint8_t > codec::generate_command(commands command, bool ack)
{
  /* Take any random datagram. */
  auto packet =
      kfly_packet< datagrams::Ack, false >(command, datagrams::Ack{}, ack);

  std::vector< uint8_t > out;
  kfly_parser::encode(packet.payload, out);

  return out;
}
}
