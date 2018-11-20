/****************************************************************************
*
* Copyright (C) 2015-2016 Emil Fresk.
* All rights reserved.
*
* This file is part of the SerialPipe library.
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

#include <iostream>
#include "serialpipe.h"

using namespace std;

void test(const std::vector<uint8_t> &data);

int main()
{
    /* Create serial pipe */
    SerialPipe::SerialBridge sp("/dev/ttyUSB0", 115200, 1, false);

    /* SerialPipe usage:
     *
     * SerialPipe("Port",
     *            baudrate,
     *            timeout (in ms),              (optional, 1000 default)
     *            string data (true / false)    (optional, true default)
     *            string termination);          (optional, "\n" default)
     */

    /* Register a callback */
    sp.registerCallback(test);

    /* Open the serial port */
    sp.openPort();

    /* Transmit some data... */
    sp.serialTransmit("test test test");

    cin.get();

    /* Close port */
    sp.closePort();

    return 0;
}

/* Print incomming data... */
void test(const std::vector<uint8_t> &data)
{
    cout << "Data received: ";
    //for (const uint8_t &ch : data)
    //{
    //    cout << static_cast<char>( ch );
    //}

    cout << endl;
}

