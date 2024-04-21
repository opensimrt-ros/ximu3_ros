#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/spinner.h"
#include "ros/time.h"
#include "sensor_msgs/Imu.h"
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include "ximu3_ros/Connection_ros.h"
#include "ros/ros.h"
#include "std_msgs/Float32.h"
#include <functional>

int get_divisor(std::string param_name,ros::NodeHandle nh)
{
	int divisor_rate;
	if (nh.getParam(param_name + "_divisor", divisor_rate))
	{
		if (divisor_rate > 0)
		{
			ROS_INFO_STREAM("Setting" << param_name << " divisor rate to:"<< divisor_rate);
			ROS_INFO_STREAM("Will publish "<< param_name <<" at rate of: " << 400.0/divisor_rate);
		}
		else
		{
			ROS_ERROR_STREAM(param_name << " divisor set to zero disables the XXX. No YYYY will be published. Are you sure this is what you want?");
		}
	}
	else
	{
		divisor_rate = 8;
		ROS_WARN_STREAM("Using default "<< param_name << " divisor rate: " << divisor_rate );
	}

	return divisor_rate;
}

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

	int ahrs_divisor_rate = get_divisor("ahrs", nh);
	int sensors_divisor_rate = get_divisor("sensor", nh);

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

	bool do_calib = false;
	nh.getParam("do_calibration", do_calib);

	bool wait_to_start = false;
	nh.getParam("wait_to_start", wait_to_start);


	if (wait_to_start)
	{
		ros::Rate wait_rate(0.1);
		bool start_now = false;
		boost::function<bool(std_srvs::EmptyRequest& req, std_srvs::EmptyResponse& res)> wait_callback = [&start_now](auto req, auto res) -> bool
		{
			start_now = true;
			return true;
		};
		//create a service, wait for that service call to change state so we can start
		//
		nh.advertiseService("start_now", wait_callback);

		ROS_WARN("not_tested");
		while (!start_now)
		{
			ROS_INFO_STREAM_THROTTLE(1,"Waiting to start");
			wait_rate.sleep();
			ros::spinOnce();
		}
	}


	bool dummy_publisher=true;
	nh.getParam("dummy_publisher", dummy_publisher);
	if (dummy_publisher)
	{
		ROS_WARN_STREAM("Running dummy_publisher!!! Not attempting to connect to imu: "<<own_tf_name);
		ros::Rate rr(400.0/ahrs_divisor_rate);

		while(ros::ok())
		{
			sensor_msgs::Imu imu_msg;
			imu_msg.header.stamp = ros::Time::now();
			imu_pub.publish(imu_msg);
			rr.sleep();
			ros::spinOnce();
		}
	}
	else
	{
		Connection c( parent_frame_id, own_tf_name, ahrs_divisor_rate, sensors_divisor_rate, origin, temp_pub, bat_pub, bat_v_pub, imu_pub, publish_status, nh, do_calib);
		//c.run(ximu3::UdpConnectionInfo("192.168.1.1", 9000, 8001));
		c.run(ximu3::UdpConnectionInfo(ip_address, receive_port, send_port));
	}
	return 0;
}

