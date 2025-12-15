#include "instance.h"

#ifndef L151_DEV

#define FB       (499.2e6f)                    /* Basis frequency */
#define L_M_5    (SPEED_OF_LIGHT /FB /13.0f)   /* Lambda, m, CH5 */
#define D_M_5    (0.0204f)       /* Distance between centers of antennas, ~(L_M/2), m, CH5 */


/* RESP数据帧格式 */
static uint8_t tx_resp_msg[RESP_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0x00, 0x00, 0x00, 0x80, FUNC_CODE_RESP, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/* 接收数据buffer */
static uint8_t rx_buffer[FRAME_LEN_MAX];

/* TWR时间戳，用于计算飞行时间 */
static uint64_t poll_rx_ts;    
static uint64_t resp_tx_ts;
static uint64_t final_rx_ts;

/* 发送和接收数据中断标志 */
static volatile uint8_t rx_status = RX_WAIT;
static volatile uint8_t tx_status = TX_WAIT;

/* 保存当前ID标签的测距值，下次发送resp时发给标签 */
typedef struct {
	uint8_t range_nb;
	int32_t distance;
} prev_range_t;
static prev_range_t prev_range[MAX_TAG_LIST_SIZE];
static uint8_t resp_valid = 0x00;                              //基站数据有效标志

static uint8_t sr, rr; //用于控制当前基站处于resp时是发送还是接收
static int16_t stsqual;//STS数据接收质量

/* PDOA接收 */
static int16_t pollPDOA;
static int16_t finalPDOA;


static double pdoa2path_diff_ch5(float x);
static void pdoa2XY(void);
static float uwb_calc_aoa(float pdoa);


void anchor_app(void)
{
   switch (state)
   {
       case STA_INIT_POLL_BLINK: //初始化接收机，接收poll消息

           dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  //设置帧过滤模式开启
           dwt_setpreambledetecttimeout(0);                        //清除前导码超时，一直接收
           dwt_setrxtimeout(0);                                    //清除接收数据超时，一直接收
           int ret = dwt_rxenable(DWT_START_RX_IMMEDIATE);         //打开接收机，等待接收数据    
           if(ret == DWT_ERROR)
           {
               state = STA_INIT_POLL_BLINK;
               break;
           }           
           for(uint8_t i = 0; i < MAX_AHCHOR_NUMBER; i++)
           {
               distance_report[i] = -1;  
           }
           rx_status = RX_WAIT;                                    //清rx标志位，中断服务函数更改其状态
           state = STA_WAIT_POLL_BLINK; 
           break;
       
       case STA_WAIT_POLL_BLINK:                                   //等待poll消息，中断回调函数状态变更
       
           if(rx_status == RX_OK)                                  //接收到数据
           {
               rx_status = RX_WAIT;
               {
                   state = STA_RECV_POLL_BLINK;
               }
           }
           else if((rx_status == RX_TIMEOUT)||(rx_status == RX_ERROR))  //接收数据错误，重新开启接收
           {
               state = STA_INIT_POLL_BLINK;
           }

           break;

       case STA_RECV_POLL_BLINK://接收处理poll消息
           if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_POLL)      //判断收到的数据是POLL
           {
               range_nb = rx_buffer[RANGE_NB_IDX];             //取range_nb，resp发送时发送相同的range_nb
               recv_tag_id = rx_buffer[SENDER_SHORT_ADD_IDX];  //取发送标签的ID
               if(recv_tag_id >= inst_slot_number)             //标签ID如果大于标签总容量则退出
               {
                   state = STA_INIT_POLL_BLINK;
                   break;
               }
               range_time = portGetTickCnt();  //取得测距时间
               range_global_time = GetGlobal_Us(); //取得测距时间   
               poll_rx_ts = get_rx_timestamp_u64(); //获得poll_rx时间戳

#if defined (DEBUG)
               //anc_poll_recvd = poll_rx_ts / UUS_TO_DWT_TIME;
               anc_poll_recvd = range_time;
#endif
               sr = MAX_AHCHOR_NUMBER;
               rr = 0x01 << anc_id;
               state = STA_SORR_RESP;
               led_on(LED2);
               led_on(LED3);  
           }
           else //非POLL数据重新开启接收POLL
           {
               state = STA_INIT_POLL_BLINK;
           }

           break;

       case STA_SORR_RESP://根据基站ID按顺序进行发送或接收resp消息
           
           if(sr > 0)// sr>0 处于resp阶段，按基站ID判断接收resp或发送resp
           {
               if(rr & 0x01)//当前该基站需发送resp
               {
                   rr = 0;
                   state = STA_SEND_RESP;
               }
               else//当前该基站需接收resp
               {
                   
                   dwt_enableframefilter(DWT_FF_NOTYPE_EN); //关闭帧过滤，能够接收所有数据

                   //设置resp数据接收机开启时间
                   uint32_t resp_rx_time = (poll_rx_ts + (MAX_AHCHOR_NUMBER - sr + 1) * inst_data_interval * UUS_TO_DWT_TIME) >> 8;
#if defined (DEBUG)
                   anc_resp_sorr[MAX_AHCHOR_NUMBER - sr] = (poll_rx_ts + (MAX_AHCHOR_NUMBER - sr + 1) * inst_data_interval * UUS_TO_DWT_TIME) / UUS_TO_DWT_TIME;
#endif
                   dwt_setdelayedtrxtime(resp_rx_time);         //设置接收机开启延时时间
                   dwt_setrxtimeout(inst_resp_rx_timeout);      //设置接收数据超时时间
                   dwt_setpreambledetecttimeout(PRE_TIMEOUT);   //设置接收前导码超时时间
                   int ret = dwt_rxenable(DWT_START_RX_DELAYED);          //延时开启接收机
                   if(ret == DWT_ERROR)
                   {
                       state = STA_INIT_POLL_BLINK;
                       break;
                   }   
                   rr = rr >> 1;
                   state = STA_WAIT_RESP;
               }
               sr = sr - 1; 
           }
           else//准备接收final
           {      
               //final数据的接收机开启时间，提前100us开启

               uint64_t final_rx_time = (poll_rx_ts + inst_poll2final_time - (((inst_dataRate == DWT_BR_110K)? ANC_FINAL_RECV_FORWARD_110K : ANC_FINAL_RECV_FORWARD_6P8M) * UUS_TO_DWT_TIME));
               //uint64_t final_rx_time = (poll_rx_ts + inst_poll2final_time - debug_dly_time * UUS_TO_DWT_TIME);
               //debug_dly_time ++;

#if defined (DEBUG)
               anc_final_recv = final_rx_time / UUS_TO_DWT_TIME;
#endif
               final_rx_time = final_rx_time >> 8;
               dwt_setdelayedtrxtime((uint32)final_rx_time);  //设置接收机开启延时时间
               dwt_setrxtimeout(inst_final_rx_timeout);       //设置接收数据超时时间
               dwt_setpreambledetecttimeout(PRE_TIMEOUT);     //设置接收前导码超时时间
               int ret = dwt_rxenable(DWT_START_RX_DELAYED);            //延时开启接收机
               if(ret == DWT_ERROR)
               {
                   state = STA_INIT_POLL_BLINK;
                   break;
               }   
               state = STA_WAIT_FINAL;
           }
           break;

       case STA_SEND_RESP: //打包发送resp消息
       {
           //设置resp消息发送时间

           uint64_t resp_tx_time = (poll_rx_ts + (anc_id + 1) * (inst_data_interval * UUS_TO_DWT_TIME) + (((inst_dataRate == DWT_BR_110K)? ANC_RESP_SEND_BACK_110K : ANC_RESP_SEND_BACK_6P8M) * UUS_TO_DWT_TIME));

#if defined (DEBUG)
           anc_resp_sorr[anc_id] = resp_tx_time / UUS_TO_DWT_TIME;
#endif
           resp_tx_time = resp_tx_time >> 8;
           dwt_setdelayedtrxtime((uint32)resp_tx_time);

           /* resp数据打包 */
           tx_resp_msg[SEQ_NB_IDX] = frame_seq_nb++;
           tx_resp_msg[RANGE_NB_IDX] = range_nb;
           tx_resp_msg[SENDER_SHORT_ADD_IDX] = anc_id;
           tx_resp_msg[RECEIVER_SHORT_ADD_IDX] = recv_tag_id;
           tx_resp_msg[FUNC_CODE_IDX] = FUNC_CODE_RESP;

           //将上次的测距值打包在resp中发给标签
           {
               tx_resp_msg[RESP_MSG_PREV_DIS_IDX]   = prev_range[recv_tag_id].distance >> 24;
               tx_resp_msg[RESP_MSG_PREV_DIS_IDX+1] = prev_range[recv_tag_id].distance >> 16;
               tx_resp_msg[RESP_MSG_PREV_DIS_IDX+2] = prev_range[recv_tag_id].distance >> 8;
               tx_resp_msg[RESP_MSG_PREV_DIS_IDX+3] = prev_range[recv_tag_id].distance;
           }

        //    // 将poll_rx_ts和resp_tx_ts打包在resp中发给标签
        //     final_msg_set_ts(&tx_resp_msg[RESP_MSG_POLL_RX_TS_IDX], poll_rx_ts);
        //     resp_tx_ts = get_tx_timestamp_u64();       
        //     final_msg_set_ts(&tx_resp_msg[RESP_MSG_RESP_TX_TS_IDX], resp_tx_ts); 

           if(anc_id == 0)//A0负责校准标签时序，防冲突
           {
               int error = 0;
               int currentSlotTime = 0;
               int expectedSlotTime = 0;
               int sframePeriod_ms = inst_one_slot_time * inst_slot_number; //sframePeriod_ms 为整个TWR周期的总时间= 单slot时间*slot个数(标签总容量)
               int slotDuration_ms = inst_one_slot_time; //slotDuration_ms 为单slot时间
               int tagSleepCorrection_ms = 0;
               
               currentSlotTime = range_time % sframePeriod_ms; //currentSlotTime 当前正在通信标签的实际slot
               expectedSlotTime = recv_tag_id * slotDuration_ms;  //expectedSlotTime 当前正在通信标签应该处于的slot
               error = expectedSlotTime - currentSlotTime;  //error 计算slot差异 用于校准

               if(error < (-(sframePeriod_ms>>1))) //if error is more negative than 0.5 period, add whole period to give up to 1.5 period sleep
               {
                   tagSleepCorrection_ms = (sframePeriod_ms + error);
               }
               else //the minimum Sleep time will be 0.5 period
               {
                   tagSleepCorrection_ms = error;
               }
               tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = (tagSleepCorrection_ms >> 8) & 0xFF;//高8位存11 
               tx_resp_msg[RESP_MSG_SLEEP_COR_IDX+1] = tagSleepCorrection_ms & 0xFF;//低8位存12
           }
           else
           {
               tx_resp_msg[RESP_MSG_SLEEP_COR_IDX] = 0;
               tx_resp_msg[RESP_MSG_SLEEP_COR_IDX+1] = 0;
           }

           dwt_writetxdata(RESP_MSG_LEN + FCS_LEN, tx_resp_msg, 0); //数据写入DW3000数据缓冲区
           dwt_writetxfctrl(RESP_MSG_LEN + FCS_LEN, 0, 1); 
           tx_status = TX_WAIT; 
           int ret = dwt_starttx(DWT_START_TX_DELAYED);  //延时发送
           if(ret == DWT_ERROR)
           {
               dwt_forcetrxoff();
               state = STA_INIT_POLL_BLINK;
               break;
           }  
           while(tx_status == TX_WAIT);//等待发送结束，状态在中断回调函数中变更
           tx_status = TX_WAIT;
           state = STA_SORR_RESP;
       }
           break;

       case STA_WAIT_RESP:  //等待接收resp消息，在中断回调函数内rx_status状态变更
           if(rx_status == RX_OK)
           {
               rx_status = RX_WAIT;
               state = STA_RECV_RESP;
           }
           else if((rx_status == RX_TIMEOUT)||(rx_status == RX_ERROR))
           {
               rx_status = RX_WAIT;
               state = STA_SORR_RESP;
           }   
           break;

           case STA_RECV_RESP: //接收到其他基站的resp消息
           {
               if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_RESP)//正确接收到resp消息
               {
                   if(rx_buffer[RANGE_NB_IDX] == range_nb)//和当前测距具有相同的range_nb
                   {
                       uint8_t recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX]; //取基站ID
                       distance_report[recv_anc_id]  = (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX]   << 24;
                       distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+1] << 16;
                       distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+2] << 8;
                       distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+3];
                   }
               }
               state = STA_SORR_RESP;
           }
               break;
       

       case STA_WAIT_FINAL:  //等待接收final消息，在中断回调函数内rx_status状态变更
           if(rx_status == RX_OK)
           {
               rx_status = RX_WAIT;

               {
                   state = STA_RECV_FINAL;
               }

           }
           else if((rx_status == RX_TIMEOUT)||(rx_status == RX_ERROR))
           {
               rx_status = RX_WAIT;
               state = STA_INIT_POLL_BLINK;
               range_status = RANGE_ERROR; 
           }
           break;

       case STA_RECV_FINAL:  //接收到final消息，数据处理
           if ((rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_FINAL) && (rx_buffer[RANGE_NB_IDX] == range_nb))
           {
               resp_valid = rx_buffer[FINAL_MSG_FINAL_VALID_IDX];
               if((resp_valid >> anc_id) & 0x01) //final消息中，本基站发送的resp消息是有效的,则进行距离计算
               {
                   uint32_t poll_tx_ts, resp_rx_ts, final_tx_ts;
                   uint32_t poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
                   double Ra, Rb, Da, Db;
                   int64_t tof_dtu;
                   double tof;


                   resp_tx_ts = get_tx_timestamp_u64();   //取得resp_tx时间戳
                   final_rx_ts = get_rx_timestamp_u64();  //取得final_rx时间戳

#if defined (DEBUG)
                   //anc_final_recvd = final_rx_ts / UUS_TO_DWT_TIME;
                   anc_final_recvd = portGetTickCnt();
#endif

                   /* 从final消息中，取得poll_tx时间戳，resp_rx时间戳，final_tx时间戳 */
                   final_msg_get_ts(&rx_buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
                   final_msg_get_ts(&rx_buffer[FINAL_MSG_RESP1_RX_TS_IDX + anc_id * FINAL_MSG_TS_LEN], &resp_rx_ts);
                   final_msg_get_ts(&rx_buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

                   /* 计算飞行时间 */
                   poll_rx_ts_32 = (uint32_t)poll_rx_ts;
                   resp_tx_ts_32 = (uint32_t)resp_tx_ts;
                   final_rx_ts_32 = (uint32_t)final_rx_ts;
                   Ra = (double)(resp_rx_ts - poll_tx_ts);
                   Rb = (double)(final_rx_ts_32 - resp_tx_ts_32);
                   Da = (double)(final_tx_ts - resp_rx_ts);
                   Db = (double)(resp_tx_ts_32 - poll_rx_ts_32);
                   tof_dtu = (int64_t)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

                   tof = (int32)tof_dtu; 
                   if (tof > 0x7FFFFFFF) 
                   {
                       tof -= 0x80000000;  
                   }

                   tof = tof * DWT_TIME_UNITS;
                   distance_now_m = tof * SPEED_OF_LIGHT;

                   if(distance_now_m > 20000.000)  
                   {
                       distance_now_m = -1;
                   }

                   distance_now_m = distance_now_m - (float)distance_offset_cm/100.0f;


                   //将上次的测距值写入distance_report用于串口输出
                   distance_report[anc_id] = prev_range[recv_tag_id].distance;
                   
                   //更新prev_range为本次测距值
                   prev_range[recv_tag_id].distance = distance_now_m * 1000;//单位转换为mm
                   prev_range[recv_tag_id].range_nb = range_nb;

                   valid_report = resp_valid;



                   dwt_rxdiag_t rx_diag;
                   dwt_readdiagnostics(&rx_diag);//读取信号强度等诊断信息
                   rx_power = rx_diag.rxPower;

                   if(distance_report[anc_id] > 0)
                   {
                       range_status = RANGE_TWR_OK;  //设置TWR成功测距标志，在dw_main.c里判断打包串口输出
                       led_off(LED2);
                       led_off(LED3); 
                   }  
               }
           }
           dwt_forcetrxoff();
           state = STA_INIT_POLL_BLINK;
           break;
       
       default:
           break;
   }
   
}


/*! ------------------------------------------------------------------------------------------------------------------
* @fn anc_rx_ok_cb()
*
* @brief Callback to process RX good frame events
*
* @param  cb_data  callback data
*
* @return  none
*/
void anc_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
   rx_status = RX_OK;

   if (cb_data->datalength <= FRAME_LEN_MAX)  //接收数据
   {
       dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
   }

   UNUSED(cb_data);

}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn anc_rx_to_cb()
*
* @brief Callback to process RX timeout events
*
* @param  cb_data  callback data
*
* @return  none
*/
void anc_rx_to_cb(const dwt_cb_data_t *cb_data)
{
   rx_status = RX_TIMEOUT;
   UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn anc_rx_err_cb()
*
* @brief Callback to process RX error events
*
* @param  cb_data  callback data
*
* @return  none
*/
void anc_rx_err_cb(const dwt_cb_data_t *cb_data)
{
   rx_status = RX_ERROR;
   UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
* @fn anc_tx_conf_cb()
*
* @brief Callback to process TX confirmation events
*
* @param  cb_data  callback data
*
* @return  none
*/
void anc_tx_conf_cb(const dwt_cb_data_t *cb_data)
{

   tx_status = TX_OK;
   UNUSED(cb_data);

}

#endif
