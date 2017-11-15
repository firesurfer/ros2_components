#pragma once

/*Qt*/
#include <QHostAddress>
#include <QHostInfo>
#include <QNetworkInterface>
namespace  ros2_components {

/**
 * @brief The Networking class provides helpers methods for accessing information about network adapters
 */
class Networking
{
public:
    /**
     * @brief GetLocalIpV4
     * @return The local ipv4 address
     */
    static uint32_t getLocalIpV4();
    /**
     * @brief getLocalIpV6
     * @return  The local ipv6 address
     */
    static std::vector<uint8_t> getLocalIpV6();
};
}
