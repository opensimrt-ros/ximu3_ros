#pragma once

#include "ros/rate.h"
#include "ros/time.h"
#include "tf2/convert.h"
#include "ximu3_ros/Connection.hpp"
#include "ximu3_ros/Ximu3.hpp"
#include "ximu3_ros/Helpers_api.hpp"
#include <inttypes.h> // PRIu64
#include <iostream>
#include <stdio.h>
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <thread>

#define TIMESTAMP_FORMAT "%8" PRIu64 " us"
#define UINT32_FORMAT " %8" PRIu32
#define UINT64_FORMAT " %8" PRIu64
#define FLOAT_FORMAT " %8.3f"
#define STRING_FORMAT " \"%s\""

class Connection
{
public:
	//ros::Publisher poser_pub;
	tf2_ros::TransformBroadcaster br;
//protected:
	ros::WallTime last_time;
       	ximu3::Connection* connection;
	std::string child_frame_id;
	std::string parent_frame_id;
	unsigned int my_divisor_rate = 8;
	Connection(): child_frame_id("ximu3"), parent_frame_id("map"), my_divisor_rate(8)
	{}
	Connection(std::string parent_frame_id_, std::string child_frame_id_, unsigned int div): child_frame_id(child_frame_id_), parent_frame_id(parent_frame_id_), my_divisor_rate(div)
	{
		ROS_INFO("Parent frame_id set to %s", parent_frame_id.c_str());
		ROS_INFO("Child frame_id set to %s", child_frame_id.c_str());
	
	}
	void set_rate(unsigned int divisor)
	{

	const std::vector<std::string> rate_commands { "{\"inertialMessageRateDivisor\":0}", "{\"highGAccelerometerMessageRateDivisor\":0}","{\"ahrsMessageRateDivisor\":"+std::to_string(divisor)+"}" };
       	for (auto a:rate_commands)
		ROS_DEBUG_STREAM("Setting command:" << a);
	connection->sendCommands(rate_commands, 2, 500);
	const std::vector<std::string> apply_commands { "{\"apply\":null}" };
       	connection->sendCommands(apply_commands, 2, 500);
	ROS_DEBUG_STREAM("Applying command on IMU");
	//sometimes it doesnt work without this? I dont want to test it. Maybe it can be removed.
	const std::vector<std::string> save_command { "{\"save\":null}" };
       	connection->sendCommands(save_command, 2, 500);
	ROS_DEBUG_STREAM("Saving settings on IMU EEPROM");


	}
	void run(const ximu3::ConnectionInfo& connectionInfo)
    {
	ros::Rate r(1);
	last_time = ros::WallTime::now();
	//ros::NodeHandle n;
       	//poser_pub = n.advertise<geometry_msgs::PoseStamped>("poser", 1000);

	// Create connection
        connection = new ximu3::Connection(connectionInfo);
        connection->addDecodeErrorCallback(decodeErrorCallback);
        connection->addStatisticsCallback(statisticsCallback);
        {
            connection->addQuaternionCallback(quaternionCallback);
        }

        // Open connection
        ROS_INFO_STREAM("Connecting to " << connectionInfo.toString());
        if (connection->open() != ximu3::XIMU3_ResultOk)
        {
            ROS_ERROR_STREAM( "Unable to open connection");
            return;
        }
        ROS_INFO_STREAM("Connection successful");
        
	set_rate(my_divisor_rate);
        // Send command to strobe LED
        const std::vector<std::string> commands { "{\"strobe\":null}" };
        connection->sendCommands(commands, 2, 500);

        // Close connection
        //helpers::wait(-1);
	while(ros::ok())
	{
		r.sleep();
	}
        connection->close();
    }

private:
    std::function<void(ximu3::XIMU3_DecodeError error)> decodeErrorCallback = [](auto decode_error)
    {
         ROS_ERROR_STREAM(XIMU3_decode_error_to_string(decode_error) );
    };

    std::function<void(ximu3::XIMU3_Statistics statistics)> statisticsCallback = [](auto statistics)
    {
        /*printf(TIMESTAMP_FORMAT UINT64_FORMAT " bytes" UINT32_FORMAT " bytes/s" UINT64_FORMAT " messages" UINT32_FORMAT " messages/s" UINT64_FORMAT " errors" UINT32_FORMAT " errors/s\n",
               statistics.timestamp,
               statistics.data_total,
               statistics.data_rate,
               statistics.message_total,
               statistics.message_rate,
               statistics.error_total,
               statistics.error_rate);*/
        ROS_DEBUG_STREAM( XIMU3_statistics_to_string(statistics)); // alternative to above
    };

    std::function<void(ximu3::XIMU3_InertialMessage message)> inertialCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT " deg/s" FLOAT_FORMAT " deg/s" FLOAT_FORMAT " deg/s" FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
               message.timestamp,
               message.gyroscope_x,
               message.gyroscope_y,
               message.gyroscope_z,
               message.accelerometer_x,
               message.accelerometer_y,
               message.accelerometer_z);
        // std::cout << XIMU3_inertial_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_MagnetometerMessage message)> magnetometerCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT " a.u." FLOAT_FORMAT " a.u." FLOAT_FORMAT " a.u.\n",
               message.timestamp,
               message.x_axis,
               message.y_axis,
               message.z_axis);
        // std::cout << XIMU3_magnetometer_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_QuaternionMessage message)> quaternionCallback = [this](auto message)
    {
	ros::WallTime this_time = ros::WallTime::now();
	double execution_time = (this_time - last_time).toNSec()*1e-6;
	last_time = this_time;
	ROS_DEBUG_STREAM("Elapsed time (ms): " << execution_time << " | Rate: " << 1/execution_time*1000);
        //geometry_msgs::PoseStamped pp;

	//printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT "\n",
        //       message.timestamp,
        //       message.w_element,
        //       message.x_element,
        //       message.y_element,
        //       message.z_element);
        // std::cout << XIMU3_quaternion_message_to_string(message) << std::endl; // alternative to above
	    //tf2::Quaternion myQuaternion(message.x_element, message.y_element,message.z_element, message.w_element);
	    //geometry_msgs::Quaternion quat_msg = tf2::toMsg(myQuaternion);
	   
	    //pp.header.frame_id = "torax";
	    //pp.header.stamp = ros::Time::now();
	    //pp.pose.orientation = quat_msg;
	    //poser_pub.publish(pp);

	    geometry_msgs::TransformStamped transformStamped;
	    transformStamped.header.stamp = ros::Time::now();
	    /*transformStamped.header.frame_id = "map";
	    transformStamped.child_frame_id = "ximu3";*/
	    transformStamped.header.frame_id = parent_frame_id;
	    transformStamped.child_frame_id = child_frame_id;
	    transformStamped.transform.rotation.x = message.x_element;
	    transformStamped.transform.rotation.y = message.y_element;
	    transformStamped.transform.rotation.z = message.z_element;
	    transformStamped.transform.rotation.w = message.w_element;
//	    transformStamped.transform.setOrigin(tf2::Vector3(0.5, 0.0, 0.0) );
	    transformStamped.transform.translation.x = 0.5;	
	    transformStamped.transform.translation.y = 0.0;	
	    transformStamped.transform.translation.x = 0.3;	

	    br.sendTransform(transformStamped);

    };

    std::function<void(ximu3::XIMU3_RotationMatrixMessage message)> rotationMatrixCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT "\n",
               message.timestamp,
               message.xx_element,
               message.xy_element,
               message.xz_element,
               message.yx_element,
               message.yy_element,
               message.yz_element,
               message.zx_element,
               message.zy_element,
               message.zz_element);
        // std::cout << XIMU3_rotation_matrix_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_EulerAnglesMessage message)> eulerAnglesCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT " deg" FLOAT_FORMAT " deg" FLOAT_FORMAT " deg\n",
               message.timestamp,
               message.roll,
               message.pitch,
               message.yaw);
        // std::cout << XIMU3_euler_angles_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_LinearAccelerationMessage message)> linearAccelerationCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
               message.timestamp,
               message.w_element,
               message.x_element,
               message.y_element,
               message.z_element,
               message.x_axis,
               message.y_axis,
               message.z_axis);
        // std::cout << XIMU3_linear_acceleration_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_EarthAccelerationMessage message)> earthAccelerationCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
               message.timestamp,
               message.w_element,
               message.x_element,
               message.y_element,
               message.z_element,
               message.x_axis,
               message.y_axis,
               message.z_axis);
        // std::cout << XIMU3_earth_acceleration_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_HighGAccelerometerMessage message)> highGAccelerometerCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT " g" FLOAT_FORMAT " g" FLOAT_FORMAT " g\n",
               message.timestamp,
               message.x_axis,
               message.y_axis,
               message.z_axis);
        // std::cout << XIMU3_high_g_accelerometer_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_TemperatureMessage message)> temperatureCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT " degC\n",
               message.timestamp,
               message.temperature);
        // std::cout << XIMU3_temperature_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_BatteryMessage message)> batteryCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT " %%" FLOAT_FORMAT " V" FLOAT_FORMAT "\n",
               message.timestamp,
               message.percentage,
               message.voltage,
               message.charging_status);
        // std::cout << XIMU3_battery_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_RssiMessage message)> rssiCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT FLOAT_FORMAT " %%" FLOAT_FORMAT " dBm\n",
               message.timestamp,
               message.percentage,
               message.power);
        // std::cout << XIMU3_rssi_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_SerialAccessoryMessage message)> serialAccessoryCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT STRING_FORMAT "\n",
               message.timestamp,
               message.char_array);
        // std::cout << XIMU3_serial_accessory_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_NotificationMessage message)> notificationCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT STRING_FORMAT "\n",
               message.timestamp,
               message.char_array);
        // std::cout << XIMU3_notification_message_to_string(message) << std::endl; // alternative to above
    };

    std::function<void(ximu3::XIMU3_ErrorMessage message)> errorCallback = [](auto message)
    {
        printf(TIMESTAMP_FORMAT STRING_FORMAT "\n",
               message.timestamp,
               message.char_array);
        // std::cout << XIMU3_error_message_to_string(message) << std::endl; // alternative to above
    };
};
