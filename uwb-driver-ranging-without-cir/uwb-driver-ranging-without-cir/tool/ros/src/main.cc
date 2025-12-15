#include <iostream>
#include "serial/serial.h"
#include <fstream>
#include <chrono>
#include <thread>

#include <iostream>
#include "serial/serial.h"

const double ranging_period = 0.05; // 50ms
const int max_anchor_num = 4;

// 校验和计算函数
uint8_t calculateChecksum(uint8_t* data, int length) {
	uint8_t checksum = 0x00;
	for (int i = 0; i < length; i++) {
		checksum ^= data[i];
	}
	return checksum;
}

typedef struct{
	uint64_t timestamp;
	int16_t acc[3];
	int16_t gyro[3];
} imu_st;

typedef struct {
    uint64_t timestamp;
    uint32_t range[8];
    uint64_t timestamp_recv[8]; 
    uint64_t poll_tx_ts;                     
    uint64_t resp_rx_ts[4];
    uint64_t final_tx_ts;
    uint64_t poll_rx_ts[4];
    uint64_t resp_tx_ts[4];
	imu_st imu;
	uint64_t unix_time_yymmdd; // 年月日对应的unix时间
} uwb_st;

#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>

int main(int argc, char** argv){
	ros::init(argc, argv, "simple_uwb_driver");
	ros::NodeHandle nh;
	ros::NodeHandle private_nh("~");

	// std::cout << "size of uwb_st: " << sizeof(uwb_st) << std::endl;

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
	uint8_t data_receive[1024] = {0};
	int data_cnt = 0;
	uint8_t data_type = 0x01;
	uwb_st uwb_data;
	uwb_st last_uwb_data;
	// 创建并打开文件
	std::ofstream file("data.txt");
	if (!file.is_open()) {
		std::cout << "Failed to open file" << std::endl;
		return -1;
	}
	while(ros::ok()){
		// std::this_thread::sleep_for(std::chrono::microseconds(1));

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
			else if(state == 2 && (buffer == 0x02 || buffer == 0x01)){ // 01为测距数据，02为测距数据+CIR数据
				// 解析数据类型
				data_type = buffer;
				state++;
			}
			else if (state == 3) {
				// 保存数据
				data_receive[data_cnt] = buffer;
				data_cnt++;
				if (data_cnt == sizeof(uwb_st)) {
					state++;
				}
			}
			else if (state == 4) {
				// 解析校验和
				uint8_t checksum = buffer;
				// 验证校验和
				uint8_t calculatedChecksum = calculateChecksum(data_receive, sizeof(uwb_st));
				if (checksum == calculatedChecksum) 
				{
					// 校验和正确，处理数据
					uwb_data = *(uwb_st*)data_receive;
				}
				else {
					// 校验和错误，丢弃数据
					std::cout << "checksum error" << std::endl;
					state = 0;
					data_cnt = 0;
					continue;
				}

				if((uwb_data.timestamp - last_uwb_data.timestamp) * 1e-6 < ranging_period * 1.5){
					uwb_st uwb_data_fix = last_uwb_data;
					for(int i = 0; i < max_anchor_num; i++){
						uwb_data_fix.range[i] = uwb_data.range[i]; // 当前时刻接收到的测距值实际上是上一时刻的测距值
					}

					float acc_f32[3];
					float gyro_f32[3];
					uint8_t* buffer = (uint8_t*)&uwb_data_fix.imu.acc[0];
					int16_t acc_x   = ((uint16_t)buffer[0] << 8)  | buffer[1];
					int16_t acc_y   = ((uint16_t)buffer[2] << 8)  | buffer[3];
					int16_t acc_z   = ((uint16_t)buffer[4] << 8)  | buffer[5];
					acc_f32[0] = acc_x / 4096.0f * 9.7936f;
					acc_f32[1] = acc_y / 4096.0f * 9.7936f;
					acc_f32[2] = acc_z / 4096.0f * 9.7936f;

					buffer = (uint8_t*)&uwb_data_fix.imu.gyro[0];
					int16_t gyro_x   = ((uint16_t)buffer[0] << 8)  | buffer[1];
					int16_t gyro_y   = ((uint16_t)buffer[2] << 8)  | buffer[3];
					int16_t gyro_z   = ((uint16_t)buffer[4] << 8)  | buffer[5];
					gyro_f32[0] = gyro_x / 32.8f * 0.01745f;
					gyro_f32[1] = gyro_y / 32.8f * 0.01745f;
					gyro_f32[2] = gyro_z / 32.8f * 0.01745f;

					// 发布测距值
					std_msgs::Float64MultiArray uwb_msg;
					uwb_msg.data.resize(1 + max_anchor_num + max_anchor_num + 6); // 时间戳+测距值+接收时间戳+imu
					uwb_msg.data[0] = uwb_data_fix.timestamp * 1e-6; // 时间戳转换为秒
					uwb_msg.data[0] = uwb_msg.data[0] 
									// + 2207 * 604800 + 315964800 + 4 * 86400; // 转换为unix时间
									+ uwb_data_fix.unix_time_yymmdd; // 转换为unix时间
					for (int i = 0; i < max_anchor_num; i++) {
						if(uwb_data_fix.range[i] == 0xFFFFFFFF)
							uwb_msg.data[i + 1] = 0;
						else
							uwb_msg.data[i + 1] = uwb_data_fix.range[i] * 1e-3; // 测距值转换为米
					}
					for (int i = 0; i < max_anchor_num; i++) {
						if(uwb_data_fix.timestamp_recv[i] == 0xFFFFFFFF)
							uwb_msg.data[i + max_anchor_num + 1] = 0;
						else
							uwb_msg.data[i + max_anchor_num + 1] = uwb_data_fix.timestamp_recv[i] * 1e-6 // 测距值转换为秒
												// + 2207 * 604800 + 315964800 + 4 * 86400; // 转换为unix时间
												+ uwb_data_fix.unix_time_yymmdd;
					}
					for(int i = 0; i < 3; i++){
						uwb_msg.data[i + max_anchor_num * 2 + 1] = acc_f32[i];
						uwb_msg.data[i + max_anchor_num * 2 + 4] = gyro_f32[i];
					}
					uwb_msg.data.push_back(uwb_data_fix.imu.timestamp * 1e-6 // 测距值转换为秒
					// 							+ 2207 * 604800 + 315964800 + 4 * 86400); // 转换为unix时间
												+ uwb_data_fix.unix_time_yymmdd); // 转换为unix时间
					uwb_pub.publish(uwb_msg);
					std::cout << std::setprecision(15);
					for(int i = 0; i < uwb_msg.data.size() - max_anchor_num - 7; i++){
						std::cout << uwb_msg.data[i] << ' ';
					}
					for(int i = 0; i < max_anchor_num; i++){
						std::cout << uwb_msg.data[i + max_anchor_num + 1] - uwb_msg.data[0] << ' ';
					}

					std::cout << std::endl;
					// std::cout << acc_f32[0] << " " << acc_f32[1] << " " << acc_f32[2] << std::endl;
				}
				last_uwb_data = uwb_data;

				// 重置状态和计数器
				if(data_type == 0x01){
					state = 0;
					data_cnt = 0;
					// std::cout << "timestamp: " << uwb_data.timestamp 
					// 			<< " range: " << uwb_data.range[0] << " " << uwb_data.range[1] << " " << uwb_data.range[2] << " " << uwb_data.range[3] << " "
					// 			<< " timediff: " << uwb_data.timestamp_recv[0] - uwb_data.timestamp << " " << uwb_data.timestamp_recv[1] - uwb_data.timestamp << " " << uwb_data.timestamp_recv[2] - uwb_data.timestamp << " " << uwb_data.timestamp_recv[3] - uwb_data.timestamp
					// 			<< std::endl;
				}
				else if(data_type == 0x02){
					state = 5;
					data_cnt = 0;
				}
			}
			else{
				state = 0;
				data_cnt = 0;
			}
		}
	}
}