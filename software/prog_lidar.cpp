#include <sw/redis++/redis++.h>
#include <iostream>
#include "CYdLidar.h"
#include "lidar_lib.h"
#include <vector>

using namespace sw::redis;

/*
    DESCRIPTION: the program will be in charge of setup lidar and get brute data.
*/
    
auto redis = Redis("tcp://127.0.0.1:6379");

Lidar soyBoy;

int main()
{
    ydlidar::os_init();

    CYdLidar laser;

    int i = 0;
    // INIT THREAD PART.
    while(i<5)
    {

        //////////////////////string property/////////////////
        /// Lidar ports
        std::string port = "/dev/ttyUSB" + std::__cxx11::to_string(i);

        /// lidar port
        laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
        /// ignore array
        std::string ignore_array;
        ignore_array.clear();
        laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(),
                        ignore_array.size());

        //////////////////////int property/////////////////
        /// lidar baudrate
        int optval = 128000;
        laser.setlidaropt(LidarPropSerialBaudrate, &optval, sizeof(int));
        /// tof lidar
        optval = TYPE_TRIANGLE;
        laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
        /// device type
        optval = YDLIDAR_TYPE_SERIAL;
        laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
        /// sample rate
        optval = 5;
        laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
        /// abnormal count
        optval = 4;
        laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

        //////////////////////bool property/////////////////
        /// fixed angle resolution
        bool b_optvalue = false;
        laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
        /// rotate 180
        laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
        /// Counterclockwise
        laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
        b_optvalue = true;
        laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
        /// one-way communication
        b_optvalue = false;
        laser.setlidaropt(LidarPropSingleChannel, &b_optvalue, sizeof(bool));
        /// intensity
        b_optvalue = false;
        laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
        /// Motor DTR
        b_optvalue = true;
        laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));

        //////////////////////float property/////////////////
        /// unit: °
        float f_optvalue = 180.0f;
        laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
        f_optvalue = -180.0f;
        laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
        /// unit: m
        f_optvalue = 16.f;
        laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
        f_optvalue = 0.1f;
        laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
        /// unit: Hz
        f_optvalue = 8.f;
        laser.setlidaropt(LidarPropScanFrequency, &f_optvalue, sizeof(float));


        // initialize SDK and LiDAR
        bool ret = laser.initialize();
        if (ret) 
        {   
            //success - Start the device scanning routine which runs on a separate thread and enable motor.
            ret = laser.turnOn();
        } 
        else 
        {
            fprintf(stderr, "%s\n", laser.DescribeError());
            fflush(stderr);
        }
        
        if(ret && ydlidar::os_isOk())
        {   
            // Memorise port name.
            soyBoy.port  = port;
            soyBoy.laser = &laser;
            soyBoy.ret   = ret;
            soyBoy.is_On = true;
            break;
        }
        else
        {
            // Stop the device scanning thread and disable motor.
            laser.turnOff();
            // Uninitialize the SDK and Disconnect the LiDAR.
            laser.disconnecting();
            // try an other port.
            i+=1;
        }
    }

    // MAIN THREAD PART.
    while (soyBoy.ret && ydlidar::os_isOk()) 
    {
        LaserScan scan;
        if(soyBoy.laser->doProcessSimple(scan)) 
        {
            /* Clean variable. */
            soyBoy.sample.clear();

            /* Run all sample. */
            for(auto point : scan.points)
            {
                /* Only take the no obstruted data. */
                if(abs(point.angle)>M_PI_2+(10*M_PI/180))
                {
                    /* Init observation variable. */
                    Lidar_data observation;
                    observation.angle = -point.angle; // rad [0,2PI]
                    observation.value = point.range;  // meter 

                    /* push the sample if they are in good. */
                    if(observation.value > 0.05 && observation.value < 10)
                    {
                        soyBoy.sample.push_back(observation);
                    }
                }
            }
            publish_lidar_sample(&redis, &soyBoy);
        } 
        else 
        {
            fprintf(stderr, "Failed to get Lidar Data\n");
            fflush(stderr);
        }
    }

    return 0;
}