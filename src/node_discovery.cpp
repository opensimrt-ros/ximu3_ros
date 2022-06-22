#include "ximu3_ros/Connection_ros.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "talker");
	std::cout << "Searching for connections" << std::endl;
	const auto devices = ximu3::NetworkDiscovery::scan(2000);
	if (devices.size() == 0)
	{
		std::cout << "No UDP connections available" << std::endl;
		return -1;
	}
	std::cout << "Found " << devices[0].device_name << " - " << devices[0].serial_number << std::endl;
	Connection c;
	c.run(ximu3::UdpConnectionInfo(devices[0].udp_connection_info));
}
