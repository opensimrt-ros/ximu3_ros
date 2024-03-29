#pragma once

#include "ros/init.h"
#include "ros/publisher.h"
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
#include "std_msgs/Float32.h"
#include "sensor_msgs/Imu.h"
#include "diagnostic_msgs/DiagnosticStatus.h"
#include "diagnostic_msgs/DiagnosticArray.h"


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
       	uint64_t last_time_stamp = 0;
       	uint64_t this_time_stamp = 0;
	ximu3::Connection* connection;
	std::string child_frame_id;
	std::string imu_name;
	std::string parent_frame_id;
	unsigned int my_divisor_rate = 8;
	std::vector<double> origin;
	std::optional<tf2::Quaternion> q_cal;
	diagnostic_msgs::DiagnosticStatus d_msg;
	diagnostic_msgs::DiagnosticArray da_msg;
	//tf2::Quaternion q_cal{0,0,0,1};
	bool publish_status;
	ros::Publisher bat_pub, bat_v_pub, temp_pub, imu_pub, diags_pub;
	Connection(): child_frame_id("ximu3"), parent_frame_id("map"), my_divisor_rate(8)
	{}
	Connection(std::string parent_frame_id_, std::string child_frame_id_, unsigned int div, std::vector<double> origin_, ros::Publisher temp_pub_, ros::Publisher bat_pub_, ros::Publisher bat_v_pub_, ros::Publisher imu_pub_, bool publish_status_ , ros::NodeHandle nh): child_frame_id(child_frame_id_), parent_frame_id(parent_frame_id_), my_divisor_rate(div), origin(origin_), bat_pub(bat_pub_), bat_v_pub(bat_v_pub_), temp_pub(temp_pub_), publish_status(publish_status_), imu_pub(imu_pub_)
	{
		imu_name = child_frame_id; // maybe I should read something fancy here to better identify them
		ROS_INFO("Parent frame_id set to %s", parent_frame_id.c_str());
		ROS_INFO("Child frame_id set to %s", child_frame_id.c_str());
		diags_pub = nh.advertise<diagnostic_msgs::DiagnosticArray>("/diagnostics",1);	
		d_msg.name = imu_name;
		da_msg.header.frame_id = parent_frame_id;
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
	    //status messages
	    if (publish_status)
	    {
	    connection->addBatteryCallback(batteryCallback);
	    connection->addTemperatureCallback(temperatureCallback);
	    }
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
		ros::WallTime this_time = ros::WallTime::now();
		double execution_time = (this_time - last_time).toNSec()*1e-6;
		r.sleep();
		if (last_time_stamp == 0) // hasnt published yet
		{
			d_msg.level = diagnostic_msgs::DiagnosticStatus::STALE;
			d_msg.message = "Haven't published yet";
			ROS_WARN_STREAM(imu_name << ": "<< d_msg.message);
			diagnostic_msgs::DiagnosticArray a_diags_msg;
			a_diags_msg.status.push_back(d_msg);
			da_msg = a_diags_msg;
						  //
		}
		if (execution_time > 100) // hasnt updated timestamp
		{
			d_msg.level = diagnostic_msgs::DiagnosticStatus::STALE; // maybe error?
			d_msg.message = "Timestamp hasn't changed. Is the IMU ON? execution_time is " + std::to_string(execution_time) + "[ms]";
			ROS_WARN_STREAM(imu_name << ": "<< d_msg.message);
			diagnostic_msgs::DiagnosticArray a_diags_msg;
			a_diags_msg.status.push_back(d_msg);
			da_msg = a_diags_msg;
						  //
		}
		else if (da_msg.status.size() <1){
			d_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
			diagnostic_msgs::DiagnosticArray a_diags_msg;
			a_diags_msg.status.push_back(d_msg);
			da_msg = a_diags_msg;
		}

		da_msg.header.stamp = ros::Time::now();
		diags_pub.publish(da_msg);
		da_msg.status.clear();
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
	    this_time_stamp = message.timestamp;
	    if (last_time_stamp == 0)
		last_time_stamp = this_time_stamp; // to initialize the thing and not give an absurd amount of time delay on first duration period
	    //ROS_INFO_STREAM(this_time_stamp); //microssecond
	    
	    //the attempt to correct receiving 4 messages at the same time, however sometimes it messes it up. so removing for now!
	    //auto fake_time = ros::Time::now() + ros::Duration((double)(this_time_stamp - last_time_stamp)/1000000);
	    
	    geometry_msgs::TransformStamped transformStamped;
	    transformStamped.header.stamp = ros::Time::now();
	    //transformStamped.header.stamp = fake_time;
	    /*transformStamped.header.frame_id = "map";
	    transformStamped.child_frame_id = "ximu3";*/
	    transformStamped.header.frame_id = parent_frame_id;
	    transformStamped.child_frame_id = child_frame_id;
	    if (!q_cal)
	    {
		ROS_INFO_STREAM(XIMU3_quaternion_message_to_string(message) );
		    q_cal = tf2::Quaternion{message.x_element, message.y_element, message.z_element, message.w_element};
	    }
	    tf2::Quaternion r{message.x_element, message.y_element, message.z_element, message.w_element};
	    /*ROS_INFO_STREAM("current q_cal: " 
			    << q_cal.getX() << " , "
			    << q_cal.getY() << " , " 
			    << q_cal.getZ() << " , " 
			    << q_cal.getW() );*/
	    transformStamped.transform.rotation = tf2::toMsg(r*q_cal->inverse());
	    //transformStamped.transform.rotation = tf2::toMsg(*q_cal*r);
	    //transformStamped.transform.rotation = tf2::toMsg(q_cal->inverse()*r*( *q_cal));
	    //transformStamped.transform.rotation = tf2::toMsg(*q_cal*r*q_cal->inverse());
	    
	    //transformStamped.transform.rotation.x = message.x_element;
	    //transformStamped.transform.rotation.y = message.y_element;
	    //transformStamped.transform.rotation.z = message.z_element;
	    //transformStamped.transform.rotation.w = message.w_element;
//	    transformStamped.transform.setOrigin(tf2::Vector3(0.5, 0.0, 0.0) );
	    transformStamped.transform.translation.x = origin[0];	
	    transformStamped.transform.translation.y = origin[1];	
	    transformStamped.transform.translation.z = origin[2];	

	    br.sendTransform(transformStamped);

	    sensor_msgs::Imu imu_msg;
	    imu_msg.header = transformStamped.header;
	    imu_msg.orientation = transformStamped.transform.rotation;
	    imu_pub.publish(imu_msg);
	    ros::spinOnce();

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

    std::function<void(ximu3::XIMU3_TemperatureMessage message)> temperatureCallback = [this](auto message)
    {
        //printf(TIMESTAMP_FORMAT FLOAT_FORMAT " degC\n",
        //       message.timestamp,
        //       message.temperature);
        ROS_DEBUG_STREAM( XIMU3_temperature_message_to_string(message)); // alternative to above
	std_msgs::Float32 temperature;
	if (message.temperature > 60)
	{
		d_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
		da_msg.status.push_back(d_msg);
		da_msg.header.stamp = ros::Time::now();
		diags_pub.publish(da_msg);
		ROS_FATAL("IMU TEMPERATURE TOO HIGH!!");
		throw std::runtime_error("Temperature of IMU over 60C. Shutting off!!!");
		std::exit(EXIT_FAILURE);
	}
	
	if (message.temperature > 50)
	{
		d_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
		da_msg.status.push_back(d_msg);
		ROS_ERROR_STREAM("IMU has temperature of more than 50C!!!");
	}

	temperature.data = message.temperature;
	temp_pub.publish(temperature);
	ros::spinOnce();
    };

    std::function<void(ximu3::XIMU3_BatteryMessage message)> batteryCallback = [this](auto message)
    {
        //printf(TIMESTAMP_FORMAT FLOAT_FORMAT " %%" FLOAT_FORMAT " V" FLOAT_FORMAT "\n",
        //       message.timestamp,
        //       message.percentage,
        //       message.voltage,
        //       message.charging_status);
	std_msgs::Float32 battery_percentage,battery_voltage;
	battery_percentage.data = message.percentage;
	battery_voltage.data = message.voltage;
	bat_pub.publish(battery_percentage);
	
	if(message.percentage < 30)
	{
		ROS_WARN_STREAM("Battery about to run out on IMU: " << imu_name);
		d_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
		da_msg.status.push_back(d_msg);
	}
	bat_v_pub.publish(battery_voltage);

        ROS_DEBUG_STREAM( XIMU3_battery_message_to_string(message)); // alternative to above
	ros::spinOnce();
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
