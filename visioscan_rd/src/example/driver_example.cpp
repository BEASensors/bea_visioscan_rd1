// Copyright (c) 2021, BEA
// Copyright (c) 2021, BEA
// All rights reserved.

#include <iostream>
#include <string>
#include <thread>
#include <chrono>
#include <visioscan_rd_driver.h>

int main(int argc, char **argv)
{
    std::cout << "Hello world!" << std::endl;
    std::string scanner_ip("192.168.1.2");
    bea_power::VISIOSCANDriver driver;

    for( int i=0; i<2; i++ )
    {
        std::cout << "Connecting to scanner at " << scanner_ip << " ... ";
        if (driver.connect(scanner_ip, 80))
        {
            std::cout << "OK" << std::endl;
        }
        else
        {
            std::cout << "FAILED!" << std::endl;
            std::cerr << "Connection to scanner at " << scanner_ip << " failed!" << std::endl;
            return 1;
        }

        auto params = driver.getParameters();
        std::cout << "Current scanner settings:" << std::endl;
        std::cout << "============================================================" << std::endl;
        // TODO:
        std::cout << "============================================================" << std::endl;


        // Start capturing scanner data
        //-------------------------------------------------------------------------
        std::cout << "Starting capturing: ";
        if (driver.startCapturingUDP())
        {
            std::cout << "OK" << std::endl;
        }
        else
        {
            std::cout << "FAILED!" << std::endl;
            return 1;
        }

        for (int s = 0; s < 5; s++)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            int scans_captured = 0;
            int scans_available = driver.getFullScansAvailable();
            for (int i = 0; i < scans_available; i++)
            {
                auto scandata = driver.getScan();
                scans_captured++;
            }
            std::cout << "Received " << scans_captured << " from scanner" << std::endl;
        }

        std::cout << "Trying to stop capture" << std::endl;

        std::cout << "Stopping capture: " << driver.stopCapturing() << std::endl;

    }

    std::cout << "Goodbye world!" << std::endl;
    return 0;
}
