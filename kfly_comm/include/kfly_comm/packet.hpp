//          Copyright Emil Fresk 2016 - 2017
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <cstdint>
#include <vector>
#include "kfly_comm/crc.hpp"
#include "kfly_comm/serializable_datagram.hpp"
#include "kfly_comm/commands.hpp"

namespace kfly_comm
{
/**
 * @brief   Takes a datagram and adds KFly header and footer to it for
 *          generating the payload.
 *
 * @tparam  Datagram      The datagram.
 * @tparam  HasDatagram   If true, then the datagram will be included in the
 *                        message.
 */
template < typename Datagram, bool HasDatagram = true >
struct kfly_packet
{
  std::vector< uint8_t > payload;

  /**
   * @brief   Constructor, takes command, datagram and ack request and fills the
   *          payload vector.
   *
   * @param[in]  command    The command,
   * @param[in]  datagram   Datagram to use.
   * @param[in]  ack        Ack request flag.
   */
  kfly_packet(commands command, const Datagram &datagram, bool ack)
  {
    /* Set correct datagram size. */
    const uint8_t size = (HasDatagram == true) ? sizeof(Datagram) : 0;
    payload.reserve(size + 4);

    /* Set ack bit if needed. */
    const uint8_t ack_bit = (ack == true) ? 0x80 : 0;

    union {
      uint16_t value;
      std::array< uint8_t, sizeof(value) > data;
    } crc;

    serializable_datagram< Datagram > serialized_datagram(datagram);
    auto datagram_array = serialized_datagram.serialize();

    /* Emplace command and size. */
    crc.value = CRC16_CCITT::generateCRC(static_cast< uint8_t >(command));
    payload.push_back(static_cast< uint8_t >(command) | ack_bit);

    crc.value = CRC16_CCITT::generateCRC(size, crc.value);
    payload.push_back(size);

    /* Emplace datagram. */
    if (HasDatagram == true)
    {
      crc.value = CRC16_CCITT::generateCRC(datagram_array, crc.value);
      std::copy(datagram_array.data(),
                datagram_array.data() + datagram_array.size(),
                back_inserter(payload));
    }

    /* Emplace CRC. */
    std::copy(crc.data.data(), crc.data.data() + crc.data.size(),
              back_inserter(payload));
  }
};
}

