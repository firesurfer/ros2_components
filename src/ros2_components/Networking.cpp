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
