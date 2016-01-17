#include "SensorLidarLMS100.h"
#include <thread>
#include <iostream>

SensorLidarLMS100::SensorLidarLMS100(uint16_t _id, bool _subscribe, std::shared_ptr<rclcpp::node::Node> parentNode):SensorLidar(_id,_subscribe,parentNode) {
    laserscanner = nullptr;
    initialized = false;
    srand(time(NULL));
    lidarIPAdress = "";
    
    ranges.resize(1081);

    lidarIPAdress = "192.168.1.2"; //TODO As Parameter

    //initDevice();
}

void SensorLidarLMS100::closeDevice() {
    if (laserscanner) {
        std::cerr << "Turn off connection to Lidar LMS100..." << std::endl;
        laserscanner->scanContinous(0);
        laserscanner->stopMeas();
        laserscanner->disconnect();
        std::cerr << "Lidar LMS100 successfully offline" << std::endl;

        delete laserscanner;
    }
}

void SensorLidarLMS100::initDevice() {
    // ...init datagram driver
    laserscanner = new LMS1xx();

    std::cerr << "SensorLidarLMS100: Try connect to laserscanner..." << std::endl;

    laserscanner->connect(lidarIPAdress);
    if (!laserscanner->isConnected()) {
        std::cerr << "Sick LMS100 connection failed" << std::endl;
        return;
    }
    laserscanner->login(); //login

    scanDataCfg dataCfg;

    cfg = laserscanner->getScanCfg(); // test

    dataCfg.outputChannel = 1;
    dataCfg.remission = true;
    dataCfg.resolution = 1;
    dataCfg.encoder = 0;
    dataCfg.position = false;
    dataCfg.deviceName = false;
    dataCfg.outputInterval = 1;

    laserscanner->setScanDataCfg(dataCfg);
    laserscanner->startMeas();
    laserscanner->saveConfig();


    std::cerr << "SensorLidarLMS100: Wait for Ready-Status..." << std::endl;
    status_t stat;
    do // wait for ready status
    {
        stat = laserscanner->queryStatus();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    }
    while (stat != ready_for_measurement);
    std::cerr << "SensorLidarLMS100: Catched Ready-Status" << std::endl;

    laserscanner->startDevice(); //logout
    laserscanner->scanContinous(1);

    initialized = true;
    std::cerr << "SensorLidarLMS100 successfully started" << std::endl;
}

SensorLidarLMS100::~SensorLidarLMS100() {
    closeDevice();
}


bool SensorLidarLMS100::to_updateData() {
    // perform scan
    if (initialized && laserscanner && laserscanner->isConnected()) {
        if (!laserscanner->getData(rawData)) {
            badScans++;
            if (badScans > 5) {
                std::cerr << "Something went horribly wrong, let's just reconnect..." << std::endl;
                closeDevice();
                initDevice();
            }
            return false;
        }
        // Scan successful, reset badScans counter
        badScans = 0;

        // Fill in data
        //cout << rawData.dist_len1 << endl;
        for (int i = 0; i < rawData.dist_len1; i++) {
            ranges[i] = rawData.dist1[i] * 0.001;
        }
        return true;
    }
    return false;
}

std::shared_ptr <sensor_msgs::msg::LaserScan> SensorLidarLMS100::to_LaserScanMessage() {
    auto message = std::make_shared<sensor_msgs::msg::LaserScan>();
    message->angle_min = cfg.startAngle / 180 * 3.1415926 / 10000.0;
    message->angle_max = cfg.stopAngle / 180 * 3.1415926 / 10000.0;
    message->angle_increment = cfg.angleResolution / 180 * 3.1415926 / 10000.0;
    message->time_increment = 0; //TODO
    message->scan_time = 1 / (cfg.scaningFrequency / 100.0);
    message->range_min = 0.1; //TODO
    message->range_max = 20;
    message->ranges = ranges;
    return message;
}
