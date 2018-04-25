/*
 * Copyright 2018 <Lennart Nachtigall> <firesurfer127@gmail.com>
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "Networking.h"


namespace  ros2_components {
uint32_t Networking::getLocalIpV4()
{
    uint32_t ipAddr = 0;
    foreach(const QNetworkInterface &interface, QNetworkInterface::allInterfaces())
    {
        if(!interface.name().contains("vmnet"))
        {
            foreach (const QHostAddress &address, interface.allAddresses())
            {
                if (address.protocol() == QAbstractSocket::IPv4Protocol && address != QHostAddress(QHostAddress::LocalHost))
                {
                    //LOG(Debug) << "Ip address is:" << address.toString().toStdString() << std::endl;
                    if(!address.isLoopback())
                    {
                        ipAddr = address.toIPv4Address();
                        break;
                    }
                }
            }
        }
    }
    return ipAddr;
}

std::vector<uint8_t> Networking::getLocalIpV6()
{
    std::vector<uint8_t> returnAddr;
    foreach(const QNetworkInterface &interface, QNetworkInterface::allInterfaces())
    {
        if(!interface.name().contains("vmnet"))
        {
            foreach (const QHostAddress &address, interface.allAddresses())
            {
                if (address.protocol() == QAbstractSocket::IPv6Protocol && address != QHostAddress(QHostAddress::LocalHost))
                {
                    //LOG(Debug) << "Ip address is:" << address.toString().toStdString() << std::endl;
                    if(!address.isLoopback())
                    {
                        Q_IPV6ADDR ipAddr = address.toIPv6Address();
                        for(int i = 0; i < 16;i++)
                        {
                            returnAddr.push_back(ipAddr[i]);
                        }
                        break;
                    }
                }
            }
        }
    }
    return returnAddr;
}
}
