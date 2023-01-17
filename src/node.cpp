#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/spinner.h"
#include "ximu3_ros/Connection_ros.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"

int main(int argc, char** argv)
{

	ros::init(argc, argv, "imu_reader_node", ros::init_options::AnonymousName);
	//if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
	//	ros::console::notifyLoggerLevelsChanged();
	//}
	ros::NodeHandle nh("~");
	std::string parent_frame_id;
	std::vector<double> origin;
	bool publish_status;
	ros::Publisher bat_pub, bat_v_pub, temp_pub, imu_pub;
	if(nh.getParam("parent_frame_id", parent_frame_id))
	{
		ROS_INFO("Using parent frame_id: %s", parent_frame_id.c_str());
	}
	else
	{
		//parent_frame_id = "world";
		parent_frame_id = "map";
	}

	std::string own_tf_name;
	if(nh.getParam("name", own_tf_name))
	{
		ROS_INFO("Publishing child_frame_id: %s", own_tf_name.c_str());
	}
	else
	{
		own_tf_name = "ximu";
		ROS_ERROR("Failed to get param 'name'. Setting default child_frame_id to '%s'.", own_tf_name.c_str());
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
	
	int ahrs_divisor_rate;
	if (nh.getParam("ahrs_divisor", ahrs_divisor_rate))
	{
		if (ahrs_divisor_rate > 0)
		{		
			ROS_INFO("Setting AHRS divisor rate to: %d", ahrs_divisor_rate);
			ROS_INFO_STREAM("Will publish TFs at rate of: " << 400.0/ahrs_divisor_rate);
		}
		else
		{
			ROS_ERROR_STREAM("AHRS divisor set to zero disables the AHRS. No Quaternions will be published. Are you sure this is what you want?");
		}
	}
	else
	{
		ahrs_divisor_rate = 8;
		ROS_WARN("Using default AHRS divisor rate: %d", ahrs_divisor_rate );
	}

	nh.getParam("origin", origin);
	if (origin.size() == 0)
	{
		origin = {0,0,0};
		ROS_INFO("Origin not set. Using 0,0,0.");
	}

	if (nh.getParam("publish_status", publish_status))
	{
		ROS_INFO("Publishing status messages");
		temp_pub = nh.advertise<std_msgs::Float32>("temperature",1);
		bat_pub = nh.advertise<std_msgs::Float32>("battery/percentage",1);
		bat_v_pub = nh.advertise<std_msgs::Float32>("battery/voltage",1);
	}
	imu_pub = nh.advertise<sensor_msgs::Imu>("imu",1);


	Connection c( parent_frame_id, own_tf_name, ahrs_divisor_rate, origin, temp_pub, bat_pub, bat_v_pub, imu_pub, publish_status);

	//c.run(ximu3::UdpConnectionInfo("192.168.1.1", 9000, 8001));	
	c.run(ximu3::UdpConnectionInfo(ip_address, receive_port, send_port));	
	return 0;
}

