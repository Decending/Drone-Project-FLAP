/****************************************************************************
*
* Copyright (C) 2015 Emil Fresk.
* All rights reserved.
*
* This file is part of the SLIP library.
*
* GNU Lesser General Public License Usage
* This file may be used under the terms of the GNU Lesser
* General Public License version 3.0 as published by the Free Software
* Foundation and appearing in the file LICENSE included in the
* packaging of this file.  Please review the following information to
* ensure the GNU Lesser General Public License version 3.0 requirements
* will be met: http://www.gnu.org/licenses/lgpl-3.0.html.
*
* If you have questions regarding the use of this file, please contact
* Emil Fresk at emil.fresk@gmail.com.
*
****************************************************************************/

#ifndef _SLIP_H
#define _SLIP_H

/* Data includes */
#include <vector>
#include <cstdint>
#include <map>

/* Threading includes */
#include <mutex>

namespace SLIP
{

/** @brief Definition of the slip callback. */
typedef std::function<void(const std::vector<uint8_t> &)> slip_callback;

class SLIP
{
private:

    /** @brief The special characters of SLIP */
    enum slip_special_t
    {
        SLIP_END = 0xC0,
        SLIP_ESC = 0xDB,
        SLIP_ESC_END = 0xDC,
        SLIP_ESC_ESC = 0xDD
    };

    /** @brief The possible states of the parser. */
    enum slip_states_t
    {
        SLIP_STATE_AWAITING_START,
        SLIP_STATE_AWAITING_ESC,
        SLIP_STATE_RECEIVING
    };

    /** @brief State transition holder. */
    slip_states_t _state;

    /** @brief Mutex for accessing the states. */
    std::mutex _state_lock;

    /** @brief Mutex for the ID counter and the callback list. */
    std::mutex _id_cblock;

    /** @brief Vector holding the registered callbacks. */
    std::map<unsigned int, slip_callback> callbacks;

    /** @brief ID counter for the removal of subscriptions. */
    unsigned int _id;

    /** @brief Counter for parsing errors. */
    unsigned int _perr;

    /** @brief Holder of the payload while decoding. */
    std::vector<uint8_t> _payload;

    /**
     * @brief   Calls all the registered callbacks with the data payload.
     *
     * @param[in] payload   The payload to be sent.
     */
    void executeCallbacks(const std::vector<uint8_t> &payload)
    {
        /* Execute callbacks. */
        std::lock_guard<std::mutex> locker(_id_cblock);

        for (auto &cb : callbacks)
            cb.second(payload);
    }

public:
    /**
     * @brief Initializes the state machine.
     */
    SLIP()
    {
        /* Initialize the state machine. */
        _state = SLIP_STATE_AWAITING_START;

        /* Initialize the ID counter. */
        _id = 0;

        /* Initialize the error counter. */
        _perr = 0;

        /* Reserve space for the state machine's scratch pad. */
        _payload.reserve(1024);
    }

    ~SLIP()
    {
    }

    /**
     * @brief   Register a callback for packet received.
     *
     * @param[in] callback  The function to register.
     * @note    Shall be of the form void(const std::vector<uint8_t> &).
     *
     * @return  Return the ID of the callback, is used for unregistration.
     */
    unsigned int registerCallback(slip_callback callback)
    {
        std::lock_guard<std::mutex> locker(_id_cblock);

        /* Add the callback to the list. */
        //callbacks.emplace_back(slip_callback_holder(_id, callback));
        callbacks.emplace(_id, callback);

        return _id++;
    }

    /**
     * @brief   Unregister a callback from the queue.
     *
     * @param[in] id    The ID supplied from @p registerCallback.
     *
     * @return  Return true if the ID was deleted.
     */
    bool unregisterCallback(const unsigned int id)
    {
        std::lock_guard<std::mutex> locker(_id_cblock);

        /* Delete the callback with correct ID, a little ugly. */
        if (callbacks.erase(id) > 0)
            return true;
        else
            /* No match, return false. */
            return false;
    }

    /**
     * @brief   Parses a data byte and calls the callbacks if a full message
     *          has been decoded.
     * @note    The callbacks are called in the context of whom that runs the
     *          parse method, this implementation does not spawn a thread for
     *          this purpose.
     *
     * @param[in] data  Data byte to be parsed.
     */
    void parse(const uint8_t data)
    {
        std::lock_guard<std::mutex> locker(_state_lock);

        switch (_state)
        {
        case SLIP_STATE_AWAITING_START:

            if (data == SLIP_END)
            {
                /* Start of data detected. */
                _state = SLIP_STATE_RECEIVING;
                _payload.clear();
            }

            break;

        case SLIP_STATE_RECEIVING:

            if (data == SLIP_ESC)
            {
                /* An escape sequence incoming, wait for next byte. */
                _state = SLIP_STATE_AWAITING_ESC;
            }
            else if (data == SLIP_END)
            {
                /* If two END bytes come in a row (payload size is 0), assume
                   re-start to not have empty packets and to guarantee that
                   the packet structure is not corrupted. */
                if (_payload.size() > 0)
                {
                    /* End of frame detected, execute callback. */
                    _state = SLIP_STATE_AWAITING_START;

                    executeCallbacks(_payload);
                }
            }
            else
            {
                /* Data received, place in buffer. */
                _payload.emplace_back(data);
            }

            break;

        case SLIP_STATE_AWAITING_ESC:
            if (data == SLIP_ESC_END)
            {
                _state = SLIP_STATE_RECEIVING;

                /* Data byte with the value of END. */
                _payload.emplace_back(SLIP_END);
            }
            else if (data == SLIP_ESC_ESC)
            {
                _state = SLIP_STATE_RECEIVING;

                /* Data byte with the value of ESC. */
                _payload.emplace_back(SLIP_ESC);
            }
            else
            {
                /* Communication error. */
                _state = SLIP_STATE_AWAITING_START;
                _perr++;
            }
            break;

        default:
            /* Some strange error, should not come here. */
            _state = SLIP_STATE_AWAITING_START;
            _perr++;
            return;
        }
    }

    /**
     * @brief   Parses a data vector and calls the callbacks if a full
     *          message has been decoded.
     *
     * @param[in] payload   Data vector to be parsed.
     */
    void parse(const std::vector<uint8_t> &payload)
    {
        for (auto &data : payload)
            parse(data);
    }

    /**
     * @brief   Clears and resets the state machine and all counters.
     */
    void clear()
    {
        std::lock_guard<std::mutex> locker(_state_lock);

        _state = SLIP_STATE_AWAITING_START;
        _perr = 0;
    }

    /**
     * @brief   Takes a message and encodes it with SLIP.
     *
     * @param[in] msg_in    Message to be encoded.
     * @param[in] msg_out   Save location for the encoded message.
     */
    static void encode(const std::vector<uint8_t> &msg_in,
                       std::vector<uint8_t> &msg_out)
    {
        /* Check so the output is of minimum sufficient size. */
        if (msg_out.capacity() < (msg_in.size() + 2))
            msg_out.reserve(msg_in.size() + 2);

        /* Add starting condition. */
        msg_out.emplace_back(SLIP_END);

        /* Add the payload and check for reserved characters. */
        for (auto &data : msg_in)
        {
            if (data == SLIP_END)
            {
                msg_out.emplace_back(SLIP_ESC);
                msg_out.emplace_back(SLIP_ESC_END);
            }
            else if (data == SLIP_ESC)
            {
                msg_out.emplace_back(SLIP_ESC);
                msg_out.emplace_back(SLIP_ESC_ESC);
            }
            else
            {
                msg_out.emplace_back(data);
            }
        }

        /* Add the end. */
        msg_out.emplace_back(SLIP_END);
    }

    /**
     * @brief   Returns the number of receive errors that have occurred since
     *          initialization or @p clear.
     *
     * @return  The number of receive errors.
     */
    unsigned int getErrors()
    {
        return _perr;
    }

};

} // namespace SLIP

#endif
