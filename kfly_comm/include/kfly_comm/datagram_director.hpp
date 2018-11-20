//          Copyright Emil Fresk 2016 - 2017
// Distributed under the Boost Software License, Version 1.0.
//    (See accompanying file LICENSE_1_0.txt or copy at
//          http://www.boost.org/LICENSE_1_0.txt)

#pragma once

#include <vector>
#include <algorithm>
#include <tuple>
#include <mutex>
#include <exception>
#include <functional>
#include <string>

namespace details
{
/**
 * @brief   A hashing function for method pointers.
 */
template < typename T, typename R, typename... Args >
std::size_t method_ptr_hash(R (T::*f)(Args...))
{
  /* The method pointer has not the same size as a void*, make generic. */
  union {
    R (T::*pf)(Args...);
    std::array< char, sizeof(R (T::*)(Args...)) > p;
  };

  /* Save the pointer. */
  pf = f;

  /* Hash whatever is inside the pointer. */
  return std::hash< std::string >{}(std::string(p.data(), p.size()));
}

/**
 * @brief   A wrapper class for function pointers and method pointers.
 *
 * @tparam Datagram   The datagram type,
 */
template < typename Datagram >
class DatagramCallback
{
private:
  /**
   * @brief   Function object holding the callback.
   */
  std::function< void(Datagram) > _callback;

  /**
   * @brief   Void pointer
   */
  void* _target;
  std::size_t _method_hash;

public:
  /**
   * @brief   Constructor, creates the callback from a method pointer.
   *
   * @param[in] obj       Object from which the callback is defined.
   * @param[in] callback  Method pointer to register.
   *
   * @tparam Object       Type of the Object.
   */
  template < typename Object >
  DatagramCallback(Object* obj, void (Object::*callback)(Datagram))
      : _target(obj), _method_hash(method_ptr_hash(callback))
  {
    if (obj == nullptr)
      throw std::invalid_argument("Datagram callback may not be a nullptr.");

    _callback = std::bind(callback, obj, std::placeholders::_1);
  }

  /**
   * @brief   Constructor, creates the callback from a function pointer.
   *
   * @param[in] fp    Function pointer to register.
   */
  DatagramCallback(void (*fp)(Datagram))
      : _target(reinterpret_cast< void* >(fp)), _method_hash(0)
  {
    if (fp == nullptr)
      throw std::invalid_argument("Datagram callback may not be a nullptr.");

    _callback = fp;
  }

  /**
   * @brief   Equality comparison operator.
   *
   * @param[in] rhs    DatagramCallback to compare with.
   */
  bool operator==(DatagramCallback& rhs) const noexcept
  {
    if (_method_hash == 0)  // Function pointer case
      return (_target == rhs._target);
    else  // Method pointer case
      return (_target == rhs._target && _method_hash == rhs._method_hash);
  }

  /**
   * @brief   Inequality comparison operator.
   *
   * @param[in] rhs    DatagramCallback to compare with.
   */
  bool operator!=(DatagramCallback& rhs) const noexcept
  {
    if (_method_hash == 0)  // Function pointer case
      return (_target != rhs._target);
    else  // Method pointer case
      return (_target != rhs._target || _method_hash != rhs._method_hash);
  }

  /**
   * @brief   Execute the callback.
   *
   * @param[in] d     Datagram to send to the callback.
   */
  void operator()(Datagram d) const
  {
    _callback(d);
  }
};
}

/**
 * @brief     A class which registers and directs callbacks based on datagram
 *            types.
 *
 * @details   The datagram director is based on that each registered datagram
 *            type gets its own callback vector (through a little meta
 *            programming, which makes it all happen at compile time), so each
 *            callback vector has its own mutex to only lock it and not the
 *            other callbacks. So in conclusion the data structure looks like
 *            this:
 *
 *            element = pair<
 *                            vector of callbacks for specific datagram,
 *                            access mutex to the vector
 *                          >
 *
 *            callback_list = tuple<
 *                                   element< Datagram 1 >,
 *                                   element< Datagram 2 >,
 *                                   ...,
 *                                   element< Datagram N >
 *                                 >
 *
 * @note      The class is thread safe, each callback vector is protected with
 *            a mutex.
 *
 * @tparam Datagrams    Datagram types to be registered in the datagram
 *                      director for callback handling.
 */
template < typename... Datagrams >
class datagram_director
{
private:
  /**
   * @brief   Meta functions to check if a type is registered in the datagram
   *          director.
   */
  template < typename Comp, typename T, typename... Ts >
  struct exists
      : std::integral_constant< bool, std::is_same< Comp, T >::value |
                                          exists< Comp, Ts... >::value >
  {
  };

  /**
   * @brief   Meta functions to check if a type is registered in the datagram
   *          director.
   */
  template < typename Comp, typename T >
  struct exists< Comp, T >
      : std::integral_constant< bool, std::is_same< Comp, T >::value >
  {
  };

  /**
   * @brief   Check so there is atleast one datagram registered.
   */
  static_assert(sizeof...(Datagrams) > 0,
                "There must be one or more datagrams registered");

  /**
   * @brief   Function pointer alias.
   *
   * @tparam Datagram  Type of the datagram.
   */
  template < typename Datagram >
  using function_ptr = void (*)(Datagram);

  /**
   * @brief   Method pointer alias.
   *
   * @tparam Object    Type of the object.
   * @tparam Datagram  Type of the datagram.
   */
  template < typename Object, typename Datagram >
  using method_ptr = void (Object::*)(Datagram);

  /**
   * @brief   Alias for the callback wrapper.
   *
   * @tparam Datagram  Type of the datagram.
   */
  template < typename Datagram >
  using callback_wrapper = details::DatagramCallback< Datagram >;

  /**
   * @brief   Helper alias to define the callback tuple elements.
   *
   * @details Each tuple element consists of a list of function pointers to
   *          functions which takes a specific datagram as argument, and each
   *          list has a corresponding mutex to make it thread safe.
   *
   * @tparam Datagram   Type of the datagram for this tuple element.
   */
  template < typename Datagram >
  using make_element =
      std::pair< std::vector< callback_wrapper< Datagram > >, std::mutex >;

  /**
   * @brief   The tuple which contains the lists of function pointers and
   *          mutexes.
   */
  std::tuple< make_element< Datagrams >... > _callbacks;

  /**
   * @brief   Registers a callback wrapper to its corresponding datagram
   *          callback.
   *
   * @param[in] cw      The callback wrapper to register.
   *
   * @tparam Datagram   Type of the datagram for this tuple element.
   */
  template < typename Datagram >
  void register_callback(callback_wrapper< Datagram >&& callback)
  {
    /* Check to the Datagram exists in the tuple. */
    static_assert(exists< Datagram, Datagrams... >::value == true,
                  "The provided datagram is not registered.");

    /* Do a nullptr check. */
    /* Get the corresponding datagram's mutex and lock it. */
    std::lock_guard< std::mutex > lock(
        std::get< make_element< Datagram > >(_callbacks).second);

    /* Emplace the callback in the corresponding callback vector. */
    std::get< make_element< Datagram > >(_callbacks)
        .first.emplace_back(std::move(callback));
  }

  /**
   * @brief   Releases a callback wrapper from its corresponding datagram
   *          callback.
   *
   * @param[in] cw      The callback wrapper to release.
   *
   * @tparam Datagram   Type of the datagram for this tuple element.
   */
  template < typename Datagram >
  void release_callback(callback_wrapper< Datagram > cw)
  {
    /* Check to the Datagram exists in the tuple. */
    static_assert(exists< Datagram, Datagrams... >::value == true,
                  "The provided datagram is not registered.");

    /* Get the corresponding datagram's mutex and lock it. */
    std::lock_guard< std::mutex > lock(
        std::get< make_element< Datagram > >(_callbacks).second);

    /* Get the callback list, and delete the requested callback. */
    auto& callbacks = std::get< make_element< Datagram > >(_callbacks).first;

    callbacks.erase(std::remove_if(callbacks.begin(), callbacks.end(),
                                   [&](callback_wrapper< Datagram > l_cb) {
                                     return cw == l_cb;
                                   }),
                    callbacks.end());
  }

public:
  /**
   * @brief   Registers a function pointer to its corresponding datagram
   *          callback.
   *
   * @param[in] fun   The function pointer to register.
   *
   * @tparam Datagram   Type of the datagram.
   */
  template < typename Datagram >
  void register_callback(function_ptr< Datagram > fun)
  {
    register_callback(callback_wrapper< Datagram >(fun));
  }

  /**
   * @brief   Registers a function pointer to its corresponding datagram
   *          callback.
   *
   * @param[in] obj   The object pointer to register.
   * @param[in] mf    The method pointer to register.
   *
   * @tparam Obejct     Type of the object in which the callback exists.
   * @tparam Datagram   Type of the datagram.
   */
  template < typename Object, typename Datagram >
  void register_callback(Object* obj, method_ptr< Object, Datagram > mf)
  {
    register_callback(callback_wrapper< Datagram >(obj, mf));
  }

  /**
   * @brief   Releases a function pointer from its corresponding datagram
   *          callback.
   *
   * @param[in] cw      The function pointer to release.
   *
   * @tparam Datagram   Type of the datagram.
   */
  template < typename Datagram >
  void release_callback(function_ptr< Datagram > fun)
  {
    release_callback(callback_wrapper< Datagram >(fun));
  }

  /**
   * @brief   Releases a method pointer from its corresponding datagram
   *          callback.
   *
   * @param[in] obj   The object pointer to register.
   * @param[in] mf    The method pointer to register.
   *
   * @tparam Obejct     Type of the object in which the callback exists.
   * @tparam Datagram   Type of the datagram.
   */
  template < typename Object, typename Datagram >
  void release_callback(Object* obj, method_ptr< Object, Datagram > mf)
  {
    release_callback(callback_wrapper< Datagram >(obj, mf));
  }

  /**
   * @brief   Executes the callbacks related to a specific datagram.
   *
   * @param[in] data   The function pointer to release.
   *
   * @tparam Datagram   Type of the datagram for this tuple element.
   */
  template < typename Datagram >
  void execute_callback(const Datagram& data)
  {
    /* Get the corresponding datagram's mutex and lock it. */
    std::lock_guard< std::mutex > lock(
        std::get< make_element< Datagram > >(_callbacks).second);

    /* Call each callback. */
    for (auto callback : std::get< make_element< Datagram > >(_callbacks).first)
      callback(data);
  }
};
