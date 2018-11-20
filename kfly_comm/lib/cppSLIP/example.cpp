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

#include <iostream>
#include <iomanip>
#include "slip.h"

using namespace std;

void test(const std::vector<uint8_t> &data);

int main()
{
    /* A test message to encode. */
    vector<uint8_t> msg;
    vector<uint8_t> enc;
    msg.emplace_back(0x10);
    msg.emplace_back(0xDB);
    msg.emplace_back(0x20);
    msg.emplace_back(0x30);
    msg.emplace_back(0xDB);
    msg.emplace_back(0xC0);
    msg.emplace_back(0x20);
    msg.emplace_back(0xDB);

    SLIP::SLIP::encode(msg, enc);

    cout << showbase // show the 0x prefix
         << internal // fill between the prefix and the number
         << setfill('0');

    cout << "Testing encoding:" << endl;
    for (auto &ch : enc)
        cout << std::hex << int(ch) << std::dec << endl;

    /* Test parsing. */
    SLIP::SLIP slip;

    auto id = slip.registerCallback(test);

    cout << endl << "Testing parsing:" << endl;
    slip.parse(enc);

    slip.unregisterCallback(id);
    slip.parse(enc);

    cin.get();

    return 0;
}

/* Print incomming data... */
void test(const std::vector<uint8_t> &data)
{
    for (const uint8_t &v : data)
    {
        cout << std::hex << int(v) << std::dec << endl;
    }

    cout << endl;
}

