#include <stdio.h>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <serial/serial.h>
#include "geometry_msgs/Twist.h"

class MAIN_JOY {
	
	public:

		MAIN_JOY(ros::NodeHandle* nh) : n(nh)
		{
	        if(!n->getParam("fw_main_joy_node/device_name", device_name_)) {
			ROS_WARN("Could not get value of device_name parameter, using default value.");
			device_name_ = "/dev/MAINJOY";
		}
                if(!n->getParam("fw_main_joy_node/x_scale", x_scale_)) {
                        ROS_WARN("Could not get value of x_scale parameter, using default value.");
                        x_scale_ = 1.2;
                }
                if(!n->getParam("fw_main_joy_node/y_scale", y_scale_)) {
                        ROS_WARN("Could not get value of y_scale parameter, using default value.");
                        y_scale_ = 0.7;
                }

		if(!n->getParam("fw_main_joy_node/frequency", frequency_)) {
			ROS_WARN("Could not get value of frequency parameter, using default value.");
			frequency_ = 50.0;
		}
		if(!n->getParam("fw_main_joy_node/baudrate", baudrate_)) {
			ROS_WARN("Could not get value of baudrate parameter, using default value.");
			baudrate_ = 38400;
			}
		if(!n->getParam("fw_main_joy_node/cmd_vel_topic", cmd_vel_topic_)) {
			ROS_WARN("Could not get value of cmd_topic parameter, using default value.");
			cmd_vel_topic_ = "/cmd_vel";
			}
                if(!n->getParam("fw_main_joy_node/uart_debugmode", uart_debugmode_)) {
                        ROS_WARN("Could not get value of uart_debugmode parameter, using default value.");
                        uart_debugmode_ = true;
			}
    	   	cmd_pub = n->advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);
		cmd_array;
		}

void parse(const std::string& input_serial) {
    // Reset cmd_array by clearing all elements
    cmd_array.clear();

    // Find the position of the first delimiter (',')
    size_t first_delimiter_pos = input_serial.find(',');
    if (first_delimiter_pos == std::string::npos) {
        // First delimiter not found, handle accordingly
        ROS_WARN("Invalid input_serial format. First delimiter not found.");
        return;
    }

    // Extract the first data before the first delimiter
    std::string first_data_str = input_serial.substr(0, first_delimiter_pos);
    try {
        float first_data = std::stof(first_data_str);
        cmd_array.push_back(first_data);
    } catch (const std::exception& e) {
        ROS_WARN("Failed to parse first_data: %s", e.what());
        return;
    }

    // Find the position of the second delimiter ('>')
    size_t second_delimiter_pos = input_serial.find('>', first_delimiter_pos + 1);
    if (second_delimiter_pos == std::string::npos) {
        // Second delimiter not found, handle accordingly
        ROS_WARN("Invalid input_serial format. Second delimiter not found.");
        return;
    }

    // Extract the second data between the first delimiter and the second delimiter
    std::string second_data_str = input_serial.substr(first_delimiter_pos + 1, second_delimiter_pos - first_delimiter_pos - 1);
    try {
        float second_data = std::stof(second_data_str);
        cmd_array.push_back(second_data);
    } catch (const std::exception& e) {
        ROS_WARN("Failed to parse second_data: %s", e.what());
        return;
    }
}

void pub_cmd_vel() {
    if (cmd_array.size() != 2) {
        ROS_WARN("cmd_array does not contain the required number of elements.");
        return;
    }

    geometry_msgs::Twist cmd_vel;
    float linear_x = x_scale_ * cmd_array[0];
    float angular_z = y_scale_ * cmd_array[1];
    cmd_vel.linear.x = linear_x;
    cmd_vel.angular.z = angular_z;
    cmd_pub.publish(cmd_vel);
}

	std::string r_device_name() {
		return device_name_;
	}

	bool r_uart_debugmode() const {
		return uart_debugmode_;
	}

	std::string r_uart_payload() {
		std::string a1 = std::to_string(cmd_array[0]);
		std::string a2 = std::to_string(cmd_array[1]);
		return "axis_X: " + a1 + " " + "axis+Y: " + a2;
	}

	float r_frequency() const {
		return frequency_;
	}

	int r_baudrate() const {
		return baudrate_;
	}

private:
	ros::NodeHandle* n;
	ros::Publisher cmd_pub;
	std::string device_name_;
	std::string cmd_vel_topic_;
	std::vector<float> cmd_array;

	bool uart_debugmode_;

	float frequency_;
	float x_scale_;
	float y_scale_;
	int baudrate_;
};

int main(int argc, char** argv) {
	ros::init(argc, argv, "fw_main_joy_node");
	ros::NodeHandle nh;
    serial::Serial ser;
	MAIN_JOY mj = MAIN_JOY(&nh);

	float frequency = mj.r_frequency();
	int baudrate = mj.r_baudrate();

	try
	{
		ser.setPort(mj.r_device_name());
		ser.setBaudrate(baudrate);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch (serial::IOException& e)
	{
		ROS_ERROR_STREAM("Unable to open port ");
		return -1;
	}

	if (ser.isOpen()) {
		ROS_INFO("Serial Port initialized");
	}
	else {
		return -1;
	}

	ros::Rate loop_rate(frequency);

	while (ros::ok()) {

    	if(ser.available()) {
    	    std::string input_serial = ser.read(ser.available());
    	    mj.parse(input_serial);
    	    if(mj.r_uart_debugmode()) ROS_INFO("Parsed data : %s", mj.r_uart_payload().c_str());
    	    mj.pub_cmd_vel();
    	}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
