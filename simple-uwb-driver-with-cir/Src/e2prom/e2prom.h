#ifndef  __E2PROM_H_
#define  __E2PROM_H_

#include "main.h"
#include "instance.h"
//AT24C02  32pages * 8bytes = 256bytes
//L151CBU6 4K = 512bytes
#define EEP_UNIT_SIZE    8

//ANT_DLY 天线延时 存储格式 0XAA(固定)  ANT_DLY高8位    ANT_DLY低8位
#define ANT_DLY_ADDR     (0*EEP_UNIT_SIZE)
//TX_PWR_ADDR 发射功率 存储格式 0XAA(固定)  TX_PWR从高到低1、2、3、4字节
#define TX_PWR_ADDR      (1*EEP_UNIT_SIZE)
//DIS_OFFSET_ADDR 距离校准值 存储格式 0XAA(固定)  DIS_OFF从高到低1、2、3、4字节
#define DIS_OFFSET_ADDR  (2*EEP_UNIT_SIZE)
//PDOA_OFFSET_ADDR PDOA校准值 存储格式 0XAA(固定)  PDOA_OFF从高到低1、2、3、4字节
#define PDOA_OFFSET_ADDR (3*EEP_UNIT_SIZE)
//DEV_ID_ADDR 设备ID 存储格式 0XAA(固定)  DEV_ID
#define DEV_ID_ADDR      (4*EEP_UNIT_SIZE)
//SLEEP_MODE_ADDR 存储休眠模式休眠前存，开机后读取，L151用
#define SLEEP_MODE_ADDR  (5*EEP_UNIT_SIZE)
//SW8_ADDR 8位拨码 存储格式 0XAA(固定)  SW8
#define SW8_ADDR         (6*EEP_UNIT_SIZE)
//SLOT_2ON_ADDR 2号拨码on时标签最大容量设置 占2字节 存储格式 0XAA(固定)  SLOT_2ON_ADDR
#define SLOT_2ON_ADDR    (7*EEP_UNIT_SIZE)
//SLOT_2OFF_ADDR 2号拨码off时标签最大容量设置 占2字节 存储格式 0XAA(固定)  SLOT_2OFF_ADDR
#define SLOT_2OFF_ADDR   (8*EEP_UNIT_SIZE)
//ANC_COORD_ADDR基站坐标，按字符串存储 $sanccd,0,0,2,0,3.1,2,3.1,0,2,3.1,3.1,2  设置基站坐标A0.X,A0.Y,A0.Z,A1.X,A1,Y,A1,Z,A2.X,A2,Y,A2,Z,A3.X,A3,Y,A3,Z
#define ANC_COORD_ADDR   (16*EEP_UNIT_SIZE)

//基站坐标总长度
#define COORD_LENGTH     120

/* L151内部EEPROM存储地址 */
#define L151_START_ADDR         0x08080000
#define L151_ANT_DLY_ADDR       (L151_START_ADDR + ANT_DLY_ADDR)
#define L151_TX_PWR_ADDR        (L151_START_ADDR + TX_PWR_ADDR)
#define L151_DEV_ID_ADDR        (L151_START_ADDR + DEV_ID_ADDR)
#define L151_ANC_COORD_ADDR     (L151_START_ADDR + ANC_COORD_ADDR)
#define L151_SLEEP_MODE_ADDR    (L151_START_ADDR + SLEEP_MODE_ADDR)
#define L151_SW8_ADDR           (L151_START_ADDR + SW8_ADDR)
#define L151_SLOT_2ON_ADDR      (L151_START_ADDR + SLOT_2ON_ADDR)
#define L151_SLOT_2OFF_ADDR     (L151_START_ADDR + SLOT_2OFF_ADDR)

extern uint8_t USE_EXT_EEPROM;

uint8_t e2prom_Init(void);
int E2prom_Write(uint8_t addr,uint8_t*data_write,int len);
int E2prom_Read(uint8_t addr,uint8_t*data_read,int len);

uint8_t e2prom_Write_Bytes(uint16_t addr, uint8_t *data, uint16_t length);
uint8_t e2prom_Read_Bytes(uint16_t addr, uint8_t *data, uint16_t length);

void write_l151_eeprom(uint32_t addr, uint8_t* data, uint16_t len);
void read_l151_eeprom(uint32_t addr, uint8_t* data, uint16_t len);

#endif 
