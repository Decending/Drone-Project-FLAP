//          Copyright Emil Fresk 2016 - 2017
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

/* Data includes */
#include <vector>
#include <cstdint>
#include <map>
#include <memory>

/* Threading includes */
#include <mutex>

/* Library includes */
#include "cppSLIP/slip.h"
//#include "cobs/cobs.h"
#include "kfly_comm/datagram_director.hpp"
#include "kfly_comm/serializable_datagram.hpp"

/* KFly includes */
#include "kfly_comm/commands.hpp"
#include "kfly_comm/datagrams.hpp"
#include "kfly_comm/crc.hpp"
#include "kfly_comm/packet.hpp"
#include "kfly_comm/datagram_traits.hpp"

namespace kfly_comm
{
/** @brief Definition of the parser type. */
using kfly_parser = SLIP::SLIP;

class codec
{
private:
  /** @brief Mutex for the ID counter and the callback list. */
  std::mutex _id_cblock;

  /** @brief Parser for the system. */
  kfly_parser _parser;

  /** @brief Lock for the parser. */
  std::mutex _parser_lock;

  /** @brief Datagram director for the callbacks and registered datagrams. */
  datagram_director<
      datagrams::Ack, datagrams::Ping, datagrams::RunningMode,
      datagrams::ManageSubscription, datagrams::SystemStrings,
      datagrams::SystemStatus, datagrams::SetDeviceStrings,
      datagrams::MotorOverride, datagrams::ControlSignals,
      datagrams::ControllerReferences, datagrams::ControllerLimits,
      datagrams::ArmSettings, datagrams::RateControllerData,
      datagrams::AttitudeControllerData, datagrams::ChannelMix,
      datagrams::RCInputSettings, datagrams::RCOutputSettings,
      datagrams::RCValues, datagrams::IMUData, datagrams::RawIMUData,
      datagrams::IMUCalibration, datagrams::EstimationAttitude,
      datagrams::ControlFilterSettings, datagrams::ComputerControlReference,
      datagrams::MotionCaptureFrame >
      _callbacks;

  /**
   * @brief   Parses a payload and, if correct, runs executeCallbacks.
   *
   * @param[in] payload   The payload to be parsed.
   */
  void parse_packet(const std::vector< uint8_t > &payload);

  /**
   * @brief   Checks the command to apply the proper structure.
   *
   * @param[in] cmd       Command byte from the packet.
   * @param[in] payload   The payload to be parsed, without header and CRC.
   */
  void transmit_datagram(const uint8_t cmd,
                         const std::vector< uint8_t > &payload);

public:
  codec();

  ~codec();

  /**
   * @brief   Register a callback.
   *
   * @param[in] callback  The function to register.
   *
   * @note    Shall be of the form void(kfly_comm::datagrams::xxx).
   */
  template < typename Datagram >
  void register_callback(void (*callback)(Datagram))
  {
    _callbacks.register_callback(callback);
  }

  /**
   * @brief   Register a callback.
   *
   * @param[in] object    The object owning the method.
   * @param[in] callback  The method to register.
   *
   * @note    Shall be of the form void(kfly_comm::datagrams::xxx).
   */
  template < class Object, typename Datagram >
  void register_callback(Object *obj, void (Object::*callback)(Datagram))
  {
    _callbacks.register_callback(obj, callback);
  }

  /**
   * @brief   Unregister a callback from the queue.
   *
   * @param[in] callback  The function to release.
   */
  template < typename Datagram >
  void release_callback(void (*callback)(Datagram))
  {
    _callbacks.release_callback(callback);
  }

  /**
   * @brief   Unregister a callback from the queue.
   *
   * @param[in] object    The object owning the method.
   * @param[in] callback  The method to release.
   */
  template < class Object, typename Datagram >
  void release_callback(Object *obj, void (Object::*callback)(Datagram))
  {
    _callbacks.release_callback(obj, callback);
  }

  /**
   * @brief   Input function for a KFly message, goes to the SLIP parser.
   *
   * @param[in] payload   The payload to be parsed.
   */
  void parse(const uint8_t data);

  /**
   * @brief   Input function for a KFly message, goes to the SLIP parser.
   *
   * @param[in] payload   The payload to be parsed.
   */
  void parse(const std::vector< uint8_t > &payload);

  /**
   * @brief   Converts a Datagram to a byte message for transmission.
   *
   * @param[in] datagram  The Datagram payload to be converted.
   * @param[in] ack       If true, then an ack is requested.
   *
   * @return A vector that holds the generated message.
   */
  template < typename Datagram >
  static std::vector< uint8_t > generate_packet(const Datagram &datagram,
                                                bool ack = false)
  {
    auto packet = kfly_packet< Datagram, true >(
        command_traits::get_packet_command< Datagram >::value, datagram, ack);

    std::vector< uint8_t > out;
    kfly_parser::encode(packet.payload, out);

    return out;
  }

  /**
   * @brief   Generate a subscription for KFly.
   *
   * @param[in] cmd       The command to generate a subscription to.
   * @param[in] dt_ms     The time between publishes of the command.
   * @param[in] subscribe Flag to subscribe or un-subscribe.
   * @param[in] port      The port on which the subscription will be published.
   *
   * @return A vector that holds the generated message.
   */
  static std::vector< uint8_t > generate_subscribe(
      commands cmd, unsigned dt_ms, bool subscribe = true,
      enums::Ports port = enums::Ports::PORT_SAME)
  {
    kfly_comm::datagrams::ManageSubscription sub;

    sub.cmd = cmd;
    sub.delta_ms = dt_ms;
    sub.subscribe = subscribe;
    sub.port = port;

    return generate_packet(sub);
  }

  /**
   * @brief   Special case for generating an un-subscription.
   *
   * @param[in] cmd   The command to generate an un-subscription to.
   * @param[in] port  The port on which the un-subscription will have effect.
   *
   * @return A vector that holds the generated message.
   */
  static std::vector< uint8_t > generate_unsubscribe(
      commands cmd, enums::Ports port = enums::Ports::PORT_SAME)
  {
    return generate_subscribe(cmd, 0, false, port);
  }

  /**
   * @brief   Special case for clearing all subscriptions.
   *
   * @return A vector that holds the generated message.
   */
  static std::vector< uint8_t > generate_unsubscribe_all()
  {
    return generate_subscribe(commands::None, 0xffffffff, false);
  }

  /**
   * @brief   Converts a command (no datagram)to a byte message for
   *          transmission.
   *
   * @param[in] command   The command to send.
   * @param[in] ack       If true, then an ack is requested.
   *
   * @return A vector that holds the generated message.
   */
  static std::vector< uint8_t > generate_command(commands command,
                                                 bool ack = false);
};

}  // namespace KFlyTelemetry
