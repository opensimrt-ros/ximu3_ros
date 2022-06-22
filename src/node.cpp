#include "ros/init.h"
#include "ros/spinner.h"
#include "ximu3_ros/Connection_ros.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{

	ros::init(argc, argv, "imu_reader_node", ros::init_options::AnonymousName);
	Connection c;
	c.run(ximu3::UdpConnectionInfo("192.168.1.1", 9000, 8001));	
	return 0;
}

