#include "Connection_ros.h"

class UdpConnection : public Connection
{
public:
    UdpConnection()
    {
            std::cout << "Searching for connections" << std::endl;
            const auto devices = ximu3::NetworkDiscovery::scan(2000);
            if (devices.size() == 0)
            {
                std::cout << "No UDP connections available" << std::endl;
                return;
            }
            std::cout << "Found " << devices[0].device_name << " - " << devices[0].serial_number << std::endl;
            run(ximu3::UdpConnectionInfo(devices[0].udp_connection_info));
    }
};
