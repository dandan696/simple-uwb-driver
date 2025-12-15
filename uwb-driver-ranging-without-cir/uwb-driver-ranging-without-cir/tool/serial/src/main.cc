#include <iostream>
#include "serial/serial.h"
#include <fstream>

#include <iostream>
#include "serial/serial.h"

typedef struct {
    uint64_t timestamp;
    uint32_t range[8];
    uint64_t timestamp_recv[8]; 
    uint64_t poll_tx_ts;                     
    uint64_t resp_rx_ts[4];
    uint64_t final_tx_ts;
    uint64_t poll_rx_ts[4];
    uint64_t resp_tx_ts[4];
} uwb_st;


struct cir_st {
	int16_t value[2032];
};
cir_st cir[4];

// 校验和计算函数
uint8_t calculateChecksum(uint8_t* data, int length) {
	uint8_t checksum = 0x00;
	for (int i = 0; i < length; i++) {
		checksum ^= data[i];
	}
	return checksum;
}

int main(void){
	serial::Serial ser;
	std::string serial_port = "COM3";
	// int serial_baudrate = 921600;
	int serial_baudrate = 1200000;

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
	uint8_t cir_buffer[10960] = {0};
	constexpr int cir_size = 4 * 2032;
	int data_cnt = 0;
	uint64_t timestamp = 0;
	uint32_t parsedData[8];
	uwb_st* uwb_data;
	uint8_t data_type = 0x01;
	// 创建并打开文件
	std::ofstream file("data.txt");
	if (!file.is_open()) {
		std::cout << "Failed to open file" << std::endl;
		return -1;
	}
	std::ofstream rxtime("rxtime.txt");
	if (!rxtime.is_open()) {
		std::cout << "Failed to open file" << std::endl;
		return -1;
	}
	while(true){
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
					uwb_data = (uwb_st*)data_receive;
				}
				// 重置状态和计数器
				if(data_type == 0x01){
					state = 0;
					data_cnt = 0;
					std::cout << "timestamp: " << uwb_data->timestamp 
								<< " range: " << uwb_data->range[0] << " " << uwb_data->range[1] << " " << uwb_data->range[2] << " " << uwb_data->range[3] << " "
								<< " timediff: " << uwb_data->timestamp_recv[0] - uwb_data->timestamp << " " << uwb_data->timestamp_recv[1] - uwb_data->timestamp << " " << uwb_data->timestamp_recv[2] - uwb_data->timestamp << " " << uwb_data->timestamp_recv[3] - uwb_data->timestamp
								<< std::endl;
				}
				else if(data_type == 0x02){
					state = 5;
					data_cnt = 0;
				}
			}
			else if(state == 5){
				// 保存数据
				cir_buffer[data_cnt] = buffer;
				data_cnt++;
				if (data_cnt == cir_size) {
					state++;
				}
			}
			else if(state == 6){
				uint8_t checksum = buffer;
				// 验证校验和
				uint8_t calculatedChecksum = calculateChecksum(cir_buffer, cir_size);
				if (checksum == calculatedChecksum) 
				{
					// 校验和正确，处理数据
					file.open("data.txt");
					// std::cout << "checksum correct: " << buffer << std::endl; 
					// file << std::endl;
					for(int i = 0; i < 4; i++){
						int16_t* cir_ptr = (int16_t*)(cir_buffer + i * 2032);
						for(int j = 0; j < 1016; j++){
							cir[i].value[j] = cir_ptr[j];
							file << cir[i].value[j] << " ";
						}
						file << std::endl;
					}
					file.close();

				}
				// 重置状态和计数器
				state = 0;
				data_cnt = 0;
				std::cout << "timestamp: " << uwb_data->timestamp 
							// << " poll_tx_ts: " << uwb_data->poll_tx_ts * 15.65e-12 << " "
							// << " final_tx_ts: " << uwb_data->final_tx_ts * 15.65e-12 << " "
							<< " final - poll: " << (uwb_data->final_tx_ts - uwb_data->poll_tx_ts) * 15.65e-12 << " "
							<< " range: " << uwb_data->range[0] << " " << uwb_data->range[1] << " " << uwb_data->range[2] << " " << uwb_data->range[3] << " "
							// << " difftime: " << uwb_data->timestamp_recv[0] - uwb_data->timestamp << " " << uwb_data->timestamp_recv[1] - uwb_data->timestamp << " " << uwb_data->timestamp_recv[2] - uwb_data->timestamp << " " << uwb_data->timestamp_recv[3] - uwb_data->timestamp
							// 15位
							<< std::cout.precision(15) 
							<< " resp_rx_ts-poll: " << (uwb_data->resp_rx_ts[0] - uwb_data->poll_tx_ts) * 15.65e-12 << " " << (uwb_data->resp_rx_ts[1] - uwb_data->poll_tx_ts) * 15.65e-12 << " " << (uwb_data->resp_rx_ts[2] - uwb_data->poll_tx_ts) * 15.65e-12 << " " << (uwb_data->resp_rx_ts[3] - uwb_data->poll_tx_ts) * 15.65e-12 << " "
							<< std::endl;

				rxtime.open("rxtime.txt");
				rxtime.precision(15);
				rxtime << uwb_data->timestamp 
						<< " " << uwb_data->poll_tx_ts * 15.65e-12
				 		<< " " << uwb_data->resp_rx_ts[0] * 15.65e-12 << " " << uwb_data->resp_rx_ts[1] * 15.65e-12 << " " << uwb_data->resp_rx_ts[2] * 15.65e-12 << " " << uwb_data->resp_rx_ts[3] * 15.65e-12 << " " << uwb_data->final_tx_ts * 15.65e-12 << std::endl;
				rxtime.close();
			}
			else{
				state = 0;
				data_cnt = 0;
			}
		}				
}
	}