#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/spinner.h"
#include "ximu3_ros/Connection_ros.h"
#include "ros/ros.h"

int main(int argc, char** argv)
{

	ros::init(argc, argv, "imu_reader_node", ros::init_options::AnonymousName);
	ros::NodeHandle nh("~");
	std::string parent_frame_id;
	if(nh.getParam("parent_frame_id", parent_frame_id))
	{
		ROS_INFO("Using parent frame_id %s", parent_frame_id.c_str());
	}
	else
	{
		//parent_frame_id = "world";
		parent_frame_id = "map";
	}

	std::string own_tf_name;
	if(nh.getParam("name", own_tf_name))
	{
		ROS_INFO("Got name param: %s", own_tf_name.c_str());
	}
	else
	{
		own_tf_name = "ximu";
		ROS_ERROR("Failed to get param 'name'. Setting default name to '%s'.", own_tf_name.c_str());
	}
	std::string ip_address;
	if(nh.getParam("ip_address", ip_address))
	{
		ROS_INFO("Accessing imu data from %s", ip_address.c_str());
	}
	else
	{
		ip_address = "192.168.1.1";
		ROS_FATAL("Need an ip address for the imu.");
		return -1;
	}
	int receive_port, send_port;
	if(nh.getParam("receive_port", receive_port))
	{
		ROS_INFO("Reading from port: %d", receive_port);
	}
	else
	{
		receive_port = 9000;
		ROS_WARN("Using default port: %d. ", receive_port);
		ROS_WARN("Make sure port is unique and exposed in docker (both dockerfile and command line call!)");
	}
	if (nh.getParam("send_port", send_port))
	{
		ROS_INFO("Sending to port %d", send_port);
	}
	else
	{
		send_port = 8001;
		ROS_WARN("Using default send_port: %d", send_port );
		//ROS_WARN("Make sure port is unique and exposed in docker (both dockerfile and command line call!)");
	}
	Connection c(parent_frame_id, own_tf_name);

	//c.run(ximu3::UdpConnectionInfo("192.168.1.1", 9000, 8001));	
	c.run(ximu3::UdpConnectionInfo(ip_address, receive_port, send_port));	
	return 0;
}

