#ifndef __INSTANCE_H
#define __INSTANCE_H

#include <stdio.h>
#include "string.h"
#include "port.h"
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_types.h"
#include "deca_spi.h"
#include "usart.h"
#include "ssd1306.h"
#include "kalman.h"
#include "icm42605.h"
#include "e2prom.h"
#include "trilateration.h"
#include "iwdg.h"
/********定义板卡角色，只能定义一个*************************************/
//#define ULM1
#define LD150
//#define LD_PA


/***********************************************************************************************/

#define UART_PORT   huart2
#define SOFTWARE_VER     	"V51"

//#define DEBUG

#define MAX_AHCHOR_NUMBER               4      //系统内最大基站数量，取4或者8，比如实际3个取4，实际6个取8
#define MAX_TAG_NUMBER_SW2_OFF          4      //ULM1、LD150、ld600模块，2号拨码off时最大标签个数
#define MAX_TAG_NUMBER_SW2_ON           1      //ULM1、LD150、ld600模块，2号拨码on时最大标签个数

/* 天线延时 
 * 计算距离结果比实际距离小，需要增大距离，则减小这个数
 * 计算距离结果比实际距离大，需要减小距离，则增大这个数
 */                                                                                                               
#if defined(ULM1)
#define ANT_DLY                         16448
#define TX_POWER                        0x1f1f1f1f
#elif defined (LD150)
#define ANT_DLY                         16468
#define TX_POWER                        0x1f1f1f1f
#elif defined (LD_PA)
#define ANT_DLY                         16473
#define TX_POWER                        0X9C9C9C9C
#endif

#define MAX_TAG_LIST_SIZE               50    //分配最大标签数量做开尔曼滤波用

/* 数据帧超时及延时时间*/
#define PRE_TIMEOUT                     5

#if (MAX_AHCHOR_NUMBER == 4)
#define ONE_SLOT_TIME_MS_110K           28
#define ONE_SLOT_TIME_MS_6P8M           100 // ms
#elif (MAX_AHCHOR_NUMBER == 8)
#define ONE_SLOT_TIME_MS_110K           50
#define ONE_SLOT_TIME_MS_6P8M           15
#endif

// us
#define FINAL_RX_TIMEOUT_6P8M           500
#define RESP_RX_TIMEOUT_6P8M            500
#define INIT_RX_TIMEOUT_6P8M            500
#define DATA_INTERVAL_TIME_6P8M         10000     //6.8M通信速率下，相邻消息间隔时间
#define ANC_FINAL_RECV_FORWARD_6P8M     120     //6.8M通信速率下，基站接收FINAL消息前提前打开接收机的时间
#define ANC_RESP_SEND_BACK_6P8M         160     //6.8M通信速率下，基站延后发送RESP消息时间
#define ANC_INIT_SEND_DELAY_6P8         700     //6.8M通信速率下，基站发送INIT消息延时时间

#if (MAX_AHCHOR_NUMBER == 4)
#define FINAL_RX_TIMEOUT_110K           3500
#elif (MAX_AHCHOR_NUMBER == 8)
#define FINAL_RX_TIMEOUT_110K           5000
#endif
#define RESP_RX_TIMEOUT_110K            2800
#define INIT_RX_TIMEOUT_110K            2000
#define DATA_INTERVAL_TIME_110K         4000    //110K通信速率下，相邻消息间隔时间
#define ANC_FINAL_RECV_FORWARD_110K     350     //110K通信速率下，基站接收FINAL消息前提前打开接收机的时间
#define ANC_RESP_SEND_BACK_110K         500     //110K通信速率下，基站延后发送RESP消息时间
#define ANC_INIT_SEND_DELAY_110K        3000    //110K通信速率下，基站发送INIT消息延时时间

#define BLINK_INTERVAL_TIME             500

#define MAX_POLL_SEND_SLEEP_COUNT       150     //MAX_POLL_SEND_SLEEP_COUNT次发送后无运动则进入休眠
#define MAX_BLINK_SEND_SLEEP_COUNT      20      //MAX_BLINK_SEND_SLEEP_COUNT次发送后无运动则进入休眠


/* PAN ID */
#define PAN_ID                          0xDECA

/* 中断状态标志 */
#define RX_WAIT                         0
#define TX_WAIT                         0
#define RX_OK                           1
#define TX_OK                           1
#define RX_TIMEOUT                      2
#define RX_ERROR                        3


/* 数据帧长度 */
#define POLL_MSG_LEN                    11
#define RESP_MSG_LEN                    17
#define FIANL_MSG_LEN                   (20 + 4 * MAX_AHCHOR_NUMBER)
#define BLINK_MSG_LEN                   10
#define INIT_MSG_LEN                    12

/* 数据帧数组索引 */
#define SEQ_NB_IDX                      2
#define RECEIVER_SHORT_ADD_IDX          5
#define SENDER_SHORT_ADD_IDX            7
#define FUNC_CODE_IDX                   9
#define RANGE_NB_IDX                    10
#define RESP_MSG_SLEEP_COR_IDX          11
#define RESP_MSG_PREV_DIS_IDX           13
#define RESP_MSG_POLL_RX_TS_IDX         17
#define RESP_MSG_RESP_TX_TS_IDX         21

#define INIT_MSG_SLEEP_COR_IDX          10
#define FINAL_MSG_FINAL_VALID_IDX       11
#define FINAL_MSG_POLL_TX_TS_IDX        12
#define FINAL_MSG_FINAL_TX_TS_IDX       16
#define FINAL_MSG_RESP1_RX_TS_IDX       20
#define FINAL_MSG_RESP2_RX_TS_IDX       24
#define FINAL_MSG_RESP3_RX_TS_IDX       28
#define FINAL_MSG_RESP4_RX_TS_IDX       32
#define FINAL_MSG_RESP5_RX_TS_IDX       36
#define FINAL_MSG_RESP6_RX_TS_IDX       40
#define FINAL_MSG_RESP7_RX_TS_IDX       44
#define FINAL_MSG_RESP8_RX_TS_IDX       48


/*  function code */
#define FUNC_CODE_POLL                  0x21
#define FUNC_CODE_RESP                  0x10
#define FUNC_CODE_FINAL                 0x23
#define FUNC_CODE_BLINK                 0x36
#define FUNC_CODE_INIT                  0x38

/* 拨码开关键值 */
#define SWS1_IMU_MODE                   0x80	 //IMU标签 on=输出IMU数据， off=输出正常mc数据
#define SWS1_SHF_MODE                   0x40	 //默认off=10标签，100ms更新一次，on=1标签，10ms更新一次，可修改宏定义MAX_TAG_NUMBER_SW2_OFF 和 MAX_TAG_NUMBER_SW2_ON
#define SWS1_HPR_MODE                   0x20	 //外部功耗增加开关
#define SWS1_ROLE_MODE                  0x10     //工作模式0=tag 1=anchor
#define SWS1_A1A_MODE                   0x08     //anchor/tag address A1
#define SWS1_A2A_MODE                   0x04     //anchor/tag address A2
#define SWS1_A3A_MODE                   0x02     //anchor/tag address A3
#define SWS1_KAM_MODE                   0x01     //卡尔曼滤波开关

/* 状态机标志位 */
typedef enum
{
    STA_IDLE, 
    STA_SEND_POLL,
    STA_WAIT_RESP,
    STA_RECV_RESP,
    STA_SEND_FINAL,
    STA_INIT_POLL_BLINK,
    STA_WAIT_POLL_BLINK,
    STA_RECV_POLL_BLINK,
    STA_SEND_RESP,
    STA_WAIT_FINAL,
    STA_RECV_FINAL,
    STA_SORR_RESP,
    STA_SEND_BLINK,
    STA_WAIT_INIT,
    STA_RECV_INIT,
    STA_READ_CIR,
} instStatus;


/* TWR测距状态 */
typedef enum
{
    RANGE_NULL, 
    RANGE_TWR_OK,
    RANGE_ERROR
} twrStatus;

/* 系统运行角色 */
typedef enum
{
    TAG, 
    ANCHOR
} instanceModes;

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

extern uint8_t switch8;
extern uint8_t anc_id;
extern uint8_t tag_id;
extern uint8_t state;
extern int32_t distance_report[MAX_AHCHOR_NUMBER];
extern uint32_t range_time;
extern uint64_t range_global_time; 
extern uint8_t valid_report;                            //基站数据有效标志
extern uint8_t frame_seq_nb;  //每帧数据增加1
extern uint8_t range_nb;      //每次range增加1(poll resp1~4 fianl维护一套range_nb)
extern uint8_t recv_tag_id;
extern uint8_t recv_anc_id;
extern uint8_t range_status;
extern float rx_power;
extern uint16_t inst_slot_number;
extern uint8_t inst_dataRate; 
extern uint8_t inst_one_slot_time;
extern uint32_t inst_final_rx_timeout;
extern uint32_t inst_resp_rx_timeout;
extern uint32_t inst_init_rx_timeout;                          
extern uint64_t inst_poll2final_time;
extern uint32_t inst_data_interval;

extern uint64_t poll_tx_ts;                     
extern uint64_t resp_rx_ts[MAX_AHCHOR_NUMBER];
extern uint64_t final_tx_ts;

extern uint16 ant_dly;
extern double distance_now_m;  
extern int32 distance_offset_cm;                               //距离校准，单位cm

#if defined (DEBUG)
extern uint64_t tag_poll_send;                 //标签发送poll的时间（标签端TWR测距起始时间）
extern uint64_t tag_resp_recv_s[8];            //标签接收多个resp开启接收机的时间，注意不是收到的时间，是开启接收的时间
extern uint64_t tag_resp_recv_r[8];            //标签接收多个resp实际接收到的时间
extern uint64_t tag_final_send;                //标签发送final的时间


extern uint64_t anc_poll_recvd;                 //基站接收到poll的时间（基站端TWR测距起始时间）
extern uint64_t anc_resp_sorr[8];              //基站发送resp的时间或接收其他基站的resp开启接收机的时间
extern uint64_t anc_final_recv;                //基站接收final开启接收机的时间，注意不是收到的时间，是开启接收的时间
extern uint64_t anc_final_recvd;               //基站接收到final的时间

extern int debug_dly_time;
extern int32_t debug_time;
#endif


void anchor_app(void);
void tag_app(void);

void tag_rx_ok_cb(const dwt_cb_data_t *cb_data);
void tag_rx_to_cb(const dwt_cb_data_t *cb_data);
void tag_rx_err_cb(const dwt_cb_data_t *cb_data);
void tag_tx_conf_cb(const dwt_cb_data_t *cb_data);


void anc_rx_ok_cb(const dwt_cb_data_t *cb_data);
void anc_rx_to_cb(const dwt_cb_data_t *cb_data);
void anc_rx_err_cb(const dwt_cb_data_t *cb_data);
void anc_tx_conf_cb(const dwt_cb_data_t *cb_data);
void fresh_time_s(uint8_t receivedData);
uint64_t GetSysRunTimeUs(void);
uint64_t GetGlobal_Us(void);
void int32ToByteArray(int32_t value, uint8_t byteArray[4]); 
void int64ToByteArray(int64_t value, uint8_t byteArray[8]);
void floatToByteArray(float value, uint8_t byteArray[4]);
float byteArrayToFloat(uint8_t byteArray[4]);
int pack_uwb(uint8_t* data, uwb_st* uwb);   

// declare by weilinfu
extern uint64_t timestamp_send_poll;                   //标签发送poll的时间（标签端TWR测距起始时间）
extern uint64_t timestamp_recv_resp[8];                //标签接收多个resp实际接收到的时间
extern int poll_send_count;                            //标签发送poll的次数
extern uint8_t cir_buffer[];
extern int cir_buffer_start_idx;
extern const int cir_size; // 单个基站的cir数据个数
extern const int cir_buffer_size; // cir_size * sizeof(uint16_t) * 4
extern uint8_t USART_RX_BUF[];
extern uint64_t poll_rx_ts_tag[];
extern uint64_t resp_tx_ts_tag[];


#endif
