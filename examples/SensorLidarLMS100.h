#pragma once

#include "LMS1xx.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <vector>
#include <string>
#include "kamaro_common/SensorLidar.h"
using namespace KamaroModule;
class SensorLidarLMS100:public SensorLidar {
public:
    SensorLidarLMS100(uint16_t _id, bool _subscribe ,std::shared_ptr<rclcpp::node::Node> parentNode);

    virtual ~SensorLidarLMS100();

    bool to_updateData();

    std::shared_ptr <sensor_msgs::msg::LaserScan> to_LaserScanMessage();

    bool isInitialized() {
        return initialized;
    }
    virtual bool publish()
    {
        if(isSubscriber()) {
            throw std::runtime_error("Node has to be publisher to publish");
        }
        std::cout << "Publishing Message from LMS100" << std::endl;
        auto msg =to_LaserScanMessage();
        kamaroPublisher->publish(msg);

    }

private:
    void initDevice();

    void closeDevice();

    LMS1xx *laserscanner;
    bool initialized;
    scanData rawData;
    uint16_t badScans;
    std::string lidarIPAdress;
    scanCfg cfg;
    std::vector<float> ranges;

};
