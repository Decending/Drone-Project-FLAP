//          Copyright Emil Fresk 2016 - 2017
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <array>
#include <vector>
#include <type_traits>
#include <cstring>
#include <exception>

/**
 * @brief     A class which (de)serialized datagrams.
 *
 * @note      It is required that the datagram is a POD and trivially_copyable.
 *
 * @tparam Datagram   Datagram type to be handling.
 */
template < typename Datagram >
struct serializable_datagram
{
  /* POD forces a default constructor to exist. */
  static_assert(std::is_pod< Datagram >::value == true,
                "Datagram need to be POD.");

  /* Trivially copyable guarantees correct operation for std::memcpy. */
  static_assert(std::is_trivially_copyable< Datagram >::value == true,
                "Datagram need to be trivially copyable.");

  /**
   * @brief   Alias for the general serialized data type, an std::array of
   *          correct size for the datagram.
   */
  using serialized_array = std::array< uint8_t, sizeof(Datagram) >;

  /**
   * @brief   Holder of the datagram.
   */
  Datagram datagram;

  /**
   * @brief   Default constructor.
   */
  constexpr serializable_datagram() noexcept
  {
  }

  /**
   * @brief   Constructor from a datagram object.
   *
   * @param[in] datagram  The datagram to serialize.
   */
  constexpr serializable_datagram(Datagram &&datagram) noexcept
      : datagram{std::move(datagram)}
  {
  }

  /**
   * @brief   Constructor from a datagram object.
   *
   * @param[in] datagram  The datagram to serialize.
   */
  constexpr serializable_datagram(const Datagram &datagram) noexcept
      : datagram{datagram}
  {
  }

  /**
   * @brief   Constructor from a byte stream (std::array version).
   *
   * @param[in] data  The serialized data to create a datagram from.
   */
  constexpr serializable_datagram(const serialized_array &data) noexcept
  {
    std::memcpy(&datagram, data.data(), sizeof(Datagram));
  }

  /**
   * @brief   Constructor from a byte stream (std::vector version, may throw).
   *
   * @param[in] data  The serialized data to create a datagram from.
   */
  constexpr serializable_datagram(const std::vector< uint8_t > &data)
  {
    if (data.size() == sizeof(Datagram))
      std::memcpy(&datagram, data.data(), sizeof(Datagram));
    else
      throw std::invalid_argument(
          "Vector size is not the same as the datagram.");
  }

  /**
   * @brief   Serializes the datagram and returns the array.
   *
   * @return  Serialized datagram.
   */
  auto serialize() const noexcept
  {
    serialized_array buffer;
    std::memcpy(buffer.data(), &datagram, sizeof(Datagram));
    return buffer;
  }

  /**
   * @brief   Serializes the datagram and returns it in an output vector.
   *
   * @param[out]  Reference to the output vector.
   */
  constexpr void serialize(std::vector< uint8_t > &buffer) const noexcept
  {
    /* Create the serialized data and then fill the vector. */
    auto serial_data = serialize();

    buffer = std::vector< uint8_t >(serial_data.data(),
                                    serial_data.data() + serial_data.size());
  }

  /**
   * @brief   Size of the datagram.
   *
   * @return  Returns sizeof(Datagram).
   */
  static constexpr auto size() noexcept
  {
    return sizeof(Datagram);
  }

  /**
   * @brief   Returns a copy of the datagram.
   *
   * @return  Datagram reference.
   */
  constexpr auto get_datagram() const noexcept
  {
    return datagram;
  }
};
