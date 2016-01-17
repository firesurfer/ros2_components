#ifndef SENSORLIDAR_H
#define SENSORLIDAR_H

#include "ros2_components/Entity.h"

#include "sensor_msgs/msg/laser_scan.hpp"

namespace KamaroModule
{

class SensorLidar : public Entity<sensor_msgs::msg::LaserScan>
{
    typedef sensor_msgs::msg::LaserScan LocalMessageType;
public:
    SensorLidar( uint16_t _id, bool _subscribe ,std::shared_ptr<rclcpp::node::Node> parentNode) : Entity<LocalMessageType>(_id,_subscribe,parentNode, "SensorLidar")
    {

      rangesLength = 0;
   
      scanAngleRange = 0;
      scanResolution = 0;
      scanInfinity = 0;
      mountedUpSideDown = false;

      REFLECT(rangesLength);
     
      REFLECT(scanAngleRange);
      REFLECT(scanResolution);
      REFLECT(scanInfinity);
      REFLECT(mountedUpSideDown);
      
      
    }

    virtual ~SensorLidar()
    {

    }

    std::vector<float> getRanges()
    {
        return ranges;
    }
    float getScanInfinity()
    {
        return scanInfinity;
    }
    float getScanResolution()
    {
        return scanResolution;
    }
    uint16_t getScanAngleRange()
    {
        return scanAngleRange;
    }
    uint16_t getRangesLength()
    {
        return rangesLength;
    }
    bool getMountedUpSideDown()
    {
        return mountedUpSideDown;
    }
    virtual bool publish()
    {
        if(isSubscriber()) {
            throw std::runtime_error("Node has to be publisher to publish");
        }

    }
protected:

    /*Variables*/
    
    int64_t scanAngleRange;
    double scanResolution;
    double scanInfinity;
    std::vector<float> ranges;
    int64_t rangesLength;
    bool mountedUpSideDown;
    
   

private:

};

}

#endif // SENSORLIDAR_H
