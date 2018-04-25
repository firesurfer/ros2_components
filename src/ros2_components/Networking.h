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
