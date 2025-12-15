#include <iostream>
#include "serial/serial.h"
#include <fstream>
#include <chrono>
#include <thread>

#include <iostream>
#include "serial/serial.h"

// 校验和计算函数
uint8_t calculateChecksum(uint8_t* data, int length) {
	uint8_t checksum = 0x00;
	for (int i = 0; i < length; i++) {
		checksum ^= data[i];
	}
	return checksum;
}

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_uwb_driver");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	serial::Serial ser;
	std::string serial_port = "COM46";
	int serial_baudrate = 460800;
	private_nh.param<std::string>("serial_port", serial_port, serial_port);
	private_nh.param<int>("serial_baudrate", serial_baudrate, serial_baudrate);

	ROS_INFO("serial_port: %s", serial_port.c_str());
	ROS_INFO("serial_baudrate: %d", serial_baudrate);

	ros::Publisher uwb_pub = nh.advertise<std_msgs::Float64MultiArray>("uwb", 1000);

	// 打开串口
	try
	{
		ser.setPort(serial_port);
		ser.setBaudrate(serial_baudrate);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	}
	catch(serial::IOException &e)
	{
		std::cout << "Failed to open port " << std::endl;
		return -1;
	}
	std::cout << "Succeed to open port" << std::endl;

	// 清空串口缓存区
	ser.read(ser.available());

	int state = 0;
	uint8_t data_receive[256] = {0};
	int data_cnt = 0;
	int timestamp_cnt = 0;
	uint8_t timestamp_buffer[8] = {0};
	uint64_t timestamp = 0;
	uint32_t parsedData[8];
	// 创建并打开文件
	std::ofstream file("data.txt");
	if (!file.is_open()) {
		std::cout << "Failed to open file" << std::endl;
		return -1;
	}
	while(ros::ok()){
		std::this_thread::sleep_for(std::chrono::microseconds(1));
		if(ser.available()){
			uint8_t buffer = 0;
			ser.read(&buffer, 1);
			if(state == 0 && buffer == 0xAA)
			{
				state++;
			}
			else if(state == 1 && buffer == 0x55)
			{
				state++;
			}
			else if(state == 2 && buffer == 0x02){
				// 解析数据类型
				uint8_t data_type = buffer;
				state++;
			}
			else if (state == 3) {
				// 解析时间戳
				timestamp_buffer[7 - (timestamp_cnt++)] = buffer;
				if (timestamp_cnt == 8) {
					timestamp = *(uint64_t*)timestamp_buffer;
					timestamp_cnt = 0;
					state++;
				}
			}
			else if (state == 4) {
				// 解析数据
				data_receive[data_cnt] = buffer;
				data_cnt++;
				if (data_cnt == 32) {
					state++;
				}
			}
			else if (state == 5) {
				// 解析校验和
				uint8_t checksum = buffer;
				// 验证校验和
				uint8_t calculatedChecksum = calculateChecksum(data_receive, 32);
				if (checksum == calculatedChecksum) 
				{
					// 校验和正确，处理数据
					for (int i = 0; i < 8; i++) {
						parsedData[i] = (data_receive[i * 4] << 24) | (data_receive[i * 4 + 1] << 16) | (data_receive[i * 4 + 2] << 8) | data_receive[i * 4 + 3];
					}

					// 发布测距值
					std_msgs::Float64MultiArray uwb_msg;
					uwb_msg.data.resize(9); // 时间戳+8个测距值
					uwb_msg.data[0] = timestamp * 1e-6; // 时间戳转换为秒
					uwb_msg.data[0] = uwb_msg.data[0] + 2207 * 604800 + 315964800 + 4 * 86400; // 转换为unix时间
					for (int i = 0; i < 8; i++) {
						if(parsedData[i] == 0xFFFFFFFF)
							uwb_msg.data[i + 1] = 0;
						else
							uwb_msg.data[i + 1] = parsedData[i] * 1e-3; // 测距值转换为米
					}
					uwb_pub.publish(uwb_msg);
					std::cout << std::setprecision(15);
					for(int i = 0; i < uwb_msg.data.size(); i++)
						std::cout << uwb_msg.data[i] << ' ';
					std::cout << std::endl;
				}
				// 重置状态和计数器
				state = 0;
				data_cnt = 0;

			}
		}
	}
}