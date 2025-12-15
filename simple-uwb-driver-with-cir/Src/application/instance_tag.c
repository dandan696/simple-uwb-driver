#include "instance.h"
#if defined(L151_DEV)
#include "lowpower.h"
#endif

/* poll数据帧格式 */
static uint8_t tx_poll_msg[POLL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x00, FUNC_CODE_POLL, 0x00};
/* final数据帧格式 */
static uint8_t tx_final_msg[FIANL_MSG_LEN] = {0x41, 0x88, 0, 0xCA, 0xDE, 0xFF, 0xFF, 0x00, 0x00, FUNC_CODE_FINAL, 0X00};

/* 接收数据buffer */
static uint8_t rx_buffer[FRAME_LEN_MAX];

static uint8_t resp_expect = MAX_AHCHOR_NUMBER; //resp消息接收个数
static int tagSleepCorrection_ms = 0;           //标签时序校准，用于slot分配防冲突管理
static volatile uint32_t next_period_time = 0;           //标签下次测距周期开始时间，用于防冲突管理
uint8_t Correction_flag = 0;                    //该标签已经被时序校准的标志位

/* TWR时间戳，用于计算飞行时间 */
uint64_t poll_tx_ts;                     
uint64_t resp_rx_ts[MAX_AHCHOR_NUMBER];
uint64_t final_tx_ts;
uint64_t poll_rx_ts_tag[MAX_AHCHOR_NUMBER];
uint64_t resp_tx_ts_tag[MAX_AHCHOR_NUMBER];

/* 发送和接收数据中断标志 */
static volatile uint8_t rx_status = RX_WAIT;
static volatile uint8_t tx_status = TX_WAIT;

static uint8_t resp_valid = 0x00;               //基站数据有效标志
uint32 diff_time;                               //标签增加发起测距随机时间，避免冲突

uint64_t timestamp_send_poll;                   //标签发送poll的时间（标签端TWR测距起始时间）
uint64_t timestamp_recv_resp[8];                //标签接收多个resp实际接收到的时间
int poll_send_count = 0;                        //标签发送poll的次数

uint8_t cir_buffer_temp[1024] = {0};
uint8_t cir_read_flag[4] = {0};

void tag_app(void)
{
    switch (state)
    {
        
        case STA_SEND_POLL:  //打包和发送poll消息
        {
            diff_time+=(tag_id + 1); //产生随机时间避免一直冲突     
            if(diff_time > inst_one_slot_time * inst_slot_number / 2)
            {
                diff_time = 0;
            }
            range_nb++;
            /* poll数据打包 */
            tx_poll_msg[SEQ_NB_IDX] = frame_seq_nb++;  
            tx_poll_msg[RANGE_NB_IDX] = range_nb;  
            tx_poll_msg[SENDER_SHORT_ADD_IDX] = tag_id;
            tx_poll_msg[FUNC_CODE_IDX] = FUNC_CODE_POLL;
            range_time = portGetTickCnt(); //获取测距时间，用于dw_main.c中串口打包
            range_global_time = GetGlobal_Us(); //获取测距时间，用于dw_main.c中串口打包
            timestamp_send_poll = GetGlobal_Us(); //获取发送poll的时间戳，用于计算飞行时间
            dwt_writetxdata(POLL_MSG_LEN + FCS_LEN, tx_poll_msg, 0); //数据写入DW3000数据缓冲区
            dwt_writetxfctrl(POLL_MSG_LEN + FCS_LEN, 0, 1);          //配置TX帧控制寄存器
            tx_status = TX_WAIT;                                     //发送状态标志，在中断回调函数变更
            int ret = dwt_starttx(DWT_START_TX_IMMEDIATE);           //立即发送POLL消息
            if(ret == DWT_ERROR)
            {
                next_period_time = range_time + inst_one_slot_time * inst_slot_number;//设置下个周期开始时间
                state = STA_IDLE;
                break;
            }
            while(tx_status == TX_WAIT)                             //等待发送成功，tx_status在发送成功中断内变更状态
            {
                // if(portGetTickCnt() >= (range_time + 50))           //超时没有中断信号，故障，重启
                // {
                //     HAL_NVIC_SystemReset();  //重启
                // }
            }
            tx_status = TX_WAIT;                                     //清标志
            poll_tx_ts = get_tx_timestamp_u64();                     //取得poll_tx时间戳
            
#if defined (DEBUG)
            tag_poll_send = poll_tx_ts / UUS_TO_DWT_TIME;
            for(int i=0; i<MAX_AHCHOR_NUMBER; i++)
            {
                tag_resp_recv_s[i] = (poll_tx_ts + (i + 1) * inst_data_interval * UUS_TO_DWT_TIME) / UUS_TO_DWT_TIME;
            }
#endif
            resp_expect = MAX_AHCHOR_NUMBER;            //发送poll消息后，等待最大基站数量个resp消息回复
            resp_valid = 0;                             //resp有效校验初始化
            for(uint8_t i = 0; i<MAX_AHCHOR_NUMBER; i++)
            {
                distance_report[i]  = -1;  
            }
            memset(rx_buffer, 0, sizeof(rx_buffer));
            //发送POLL之后，延时开启接收等待resp
            dwt_setrxtimeout(inst_resp_rx_timeout);     //设置接收超时时间
            dwt_setpreambledetecttimeout(PRE_TIMEOUT);  //设置前导码超时
            uint32_t resp_rx_time = (poll_tx_ts + inst_data_interval * UUS_TO_DWT_TIME) >> 8;
            dwt_setdelayedtrxtime(resp_rx_time);        //设置接收机开启延时时间
            ret = dwt_rxenable(DWT_START_RX_DELAYED);   //延时开启接收机
            if(ret == DWT_ERROR)
            {
                next_period_time = range_time + inst_one_slot_time * inst_slot_number;    //设置下个周期开始时间
                state = STA_IDLE;
                break;
            }
            rx_status = RX_WAIT;
            state = STA_WAIT_RESP;
        }
            break;

        case STA_WAIT_RESP: //等待resp数据接收
        {
            if(rx_status == RX_OK)  //接收成功
            {
                state = STA_RECV_RESP;
            }
            else if((rx_status == RX_TIMEOUT) || (rx_status == RX_ERROR))//接收超时或接收错误
            {
                state = STA_RECV_RESP;
            }
            if(portGetTickCnt() >= (range_time + 50))  //超时没有中断信号，故障，重启
            {
                HAL_NVIC_SystemReset();  //重启
            }
        }
            break;

        case STA_RECV_RESP:
        {
            static uint8_t resp_recved = 0;     //单slot内接收到的resp个数，用于判定如果1个resp没收到则不发送final
            static uint8_t no_resp_count = 0;   //未接收到任何基站回复的周期计数
            if((rx_status == RX_OK) && (rx_buffer[FUNC_CODE_IDX] != FUNC_CODE_RESP))    //接收消息成功，但是消息不是resp消息则直接进入IDLE
            {
                next_period_time = range_time + inst_one_slot_time * inst_slot_number + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突      
                state = STA_IDLE;
            }
            else
            {
                if(rx_buffer[FUNC_CODE_IDX] == FUNC_CODE_RESP)      //正确接收到resp消息
                {
                    recv_anc_id = rx_buffer[SENDER_SHORT_ADD_IDX];  //取发送方基站ID
                    
                    if(rx_buffer[RANGE_NB_IDX] == range_nb)         //和poll相同的range_nb
                    {
                        resp_valid = resp_valid | (0x01 << recv_anc_id);  //设置该基站resp有效，
                        resp_rx_ts[recv_anc_id] = get_rx_timestamp_u64(); //取该基站resp_rx时间戳，用于tof计算
                        resp_recved++;
                        /* 将resp消息内的测距信息取出，用于串口数据打包输出 */
                        // ！！！！！！！！！！ 读取的是上一次的测距结果 ！！！！！！！！！
                        distance_report[recv_anc_id]  = (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX]   << 24;
                        distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+1] << 16;
                        distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+2] << 8;
                        distance_report[recv_anc_id] += (int32_t)rx_buffer[RESP_MSG_PREV_DIS_IDX+3];

                        // // 从buffer中读取poll_rx_ts，resp_tx_ts
                        // final_msg_get_ts(&rx_buffer[RESP_MSG_POLL_RX_TS_IDX], &poll_rx_ts_tag[recv_anc_id]);
                        // final_msg_get_ts(&rx_buffer[RESP_MSG_RESP_TX_TS_IDX], &resp_tx_ts_tag[recv_anc_id]);

                        uint64_t time_start = GetGlobal_Us();
                        if(cir_read_flag[recv_anc_id] == 0){
                            timestamp_recv_resp[recv_anc_id] = GetGlobal_Us(); //取该基站resp_rx时间戳

                            int reg_size_bytes = cir_size * 4; // uwb寄存器中的cir数据大小 bytes
                            int num_read_once = 1016;
                            dwt_enable_acc_clocks();
                            for(int i = 0; i < reg_size_bytes - num_read_once + 1; i += num_read_once){
                                // dwt_readaccdata(&cir_buffer_temp[0], num_read_once + 1, i); // 每次读取16字节，包含4个采样，每个采样包含两字节实部和两字节虚部
                                dwt_readaccdata_without_enableclocks(&cir_buffer_temp[0], num_read_once + 1, i); // 每次读取16字节，包含4个采样，每个采样包含两字节实部和两字节虚部
                                // 取模长
                                for(int k = 0; k < num_read_once / 4; k++){
                                    // 实部
                                    int16_t cir_real = *(int16_t *)&cir_buffer_temp[4 * k + 1]; 
                                    // 虚部
                                    int16_t cir_imag = *(int16_t *)&cir_buffer_temp[4 * k + 3];
                                    // 近似模长 = max(实部, 虚部) + 0.25 * min(实部, 虚部) 
                                    int16_t* curr = (int16_t*)&(cir_buffer[cir_buffer_start_idx 
                                                                        + recv_anc_id * cir_size * sizeof(int16_t) 
                                                                        + (i >> 1) // i / 2
                                                                        + (k << 1)] // k * 2
                                                                        );
                                    cir_real = abs(cir_real);
                                    cir_imag = abs(cir_imag);
                                    *curr = (cir_real > cir_imag) ? (cir_real + (cir_imag >> 2)) : (cir_imag + (cir_real >> 2));
                                }
                            }
                            dwt_disable_acc_clocks();
                            cir_read_flag[recv_anc_id] = 1;
                        }
                        // uint64_t time_diff = GetGlobal_Us() - time_start;
                        // int len = sprintf((char *)cir_buffer, "timeddiff: %lld\r\n", time_diff);
                        // HAL_UART_Transmit_DMA(&UART_PORT, cir_buffer, len);
                    }
                    //当收到A0基站的resp时，取tagSleepCorrection用于校准
                    if(recv_anc_id == 0)
                    {
                        tagSleepCorrection_ms = (int16) (((uint16) rx_buffer[RESP_MSG_SLEEP_COR_IDX] << 8) + rx_buffer[RESP_MSG_SLEEP_COR_IDX+1]);//高8位存11  低8位存12
                        Correction_flag = 1;
#ifndef L151_DEV

                        dwt_rxdiag_t rx_diag;
                        dwt_readdiagnostics(&rx_diag);//读取信号强度等诊断信息
						rx_power = rx_diag.rxPower;

#endif
                    }

                }
                resp_expect--;
                if(resp_expect == 0)//所有resp消息接收完成，发送final
                {
                    if(resp_recved == 0)//如果一个resp也没收到，则不发final，直接进入IDLE
                    {
                        led_off(LED3);
                        led_toggle(LED1); 
 
                        no_resp_count++;
                        if(no_resp_count > 10)
                        {
                            Correction_flag = 0;
                            no_resp_count = 0;
                        }

                        next_period_time = range_time + inst_one_slot_time * inst_slot_number + ((Correction_flag == 1)?0:(diff_time));  //如该标签未被时序校准，初次上电，增加随机时间，避免一直冲突
                        state = STA_IDLE;
                        range_status = RANGE_ERROR; 
                        
                    }
                    else //收到1个及以上resp消息
                    {
                        led_off(LED1);
                        led_toggle(LED3); 
                        state = STA_SEND_FINAL;
                        resp_recved = 0;
                        no_resp_count = 0;
                    }
                }
                else//继续接收其他resp消息
                {
                    //设置resp数据接收机开启时间
                    uint32_t resp_rx_time = (poll_tx_ts + (MAX_AHCHOR_NUMBER - resp_expect + 1) * inst_data_interval * UUS_TO_DWT_TIME) >> 8;
                    dwt_setdelayedtrxtime(resp_rx_time);  //设置接收机开启延时时间
                    int ret = dwt_rxenable(DWT_START_RX_DELAYED);   //延时开启接收机
                    // int len = sprintf((char *)cir_buffer, "resp_rx_time: %d\r\n", resp_rx_time);
                    // HAL_UART_Transmit_DMA(&UART_PORT, cir_buffer, len);
                    if(ret == DWT_ERROR)
                    {
                        next_period_time = range_time + inst_one_slot_time * inst_slot_number;//设置下个周期开始时间
                        state = STA_IDLE;
                        break;
                    }
                    // state = STA_WAIT_RESP;
                    state = STA_READ_CIR;
                }
            }
            rx_status = RX_WAIT;

        }
            break;

        case STA_READ_CIR:{

            if(rx_status == RX_OK)  //接收成功
            {
                state = STA_RECV_RESP;
            }
            else if((rx_status == RX_TIMEOUT) || (rx_status == RX_ERROR))//接收超时或接收错误
            {
                state = STA_RECV_RESP;
            }
            if(portGetTickCnt() >= (range_time + 50))  //超时没有中断信号，故障，重启
            {
                HAL_NVIC_SystemReset();  //重启
            }
        } break;

        case STA_SEND_FINAL:
        {
            uint64_t final_tx_time = (poll_tx_ts + inst_poll2final_time);  //设置final发送时间
#if defined (DEBUG)
            tag_final_send = final_tx_time / UUS_TO_DWT_TIME;
#endif
            final_tx_time = final_tx_time >> 8;
            dwt_setdelayedtrxtime((uint32)final_tx_time); //在final_tx_time这个时间发送数据
            final_tx_ts = (((uint64_t)(final_tx_time & 0xFFFFFFFEUL)) << 8) + ANT_DLY;  //final发送时间戳

            //final数据包内写入poll_tx时间戳，final_tx时间戳，4个resp_rx时间戳
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_POLL_TX_TS_IDX], poll_tx_ts);
            final_msg_set_ts(&tx_final_msg[FINAL_MSG_FINAL_TX_TS_IDX], final_tx_ts);
            for(int i = 0; i < MAX_AHCHOR_NUMBER; i++)
            {
                final_msg_set_ts(&tx_final_msg[FINAL_MSG_RESP1_RX_TS_IDX + i * FINAL_MSG_TS_LEN], resp_rx_ts[i]);
#if defined (DEBUG)
                tag_resp_recv_r[i] = resp_rx_ts[i] / UUS_TO_DWT_TIME;
#endif
            }

            //final数据打包
            tx_final_msg[SEQ_NB_IDX] = frame_seq_nb++;
            tx_final_msg[RANGE_NB_IDX] = range_nb;
            tx_final_msg[SENDER_SHORT_ADD_IDX] = tag_id;
            tx_final_msg[FINAL_MSG_FINAL_VALID_IDX] = resp_valid;
            tx_final_msg[FUNC_CODE_IDX] = FUNC_CODE_FINAL;
            dwt_writetxdata(FIANL_MSG_LEN + FCS_LEN, tx_final_msg, 0); //数据写入DW3000数据缓冲区
            dwt_writetxfctrl(FIANL_MSG_LEN + FCS_LEN, 0, 1); 

            tx_status = TX_WAIT;                              //发送状态标志，在中断回调函数变更
            int ret = dwt_starttx(DWT_START_TX_DELAYED);      //延时发送
            if(ret == DWT_ERROR)
            {
                next_period_time = range_time + inst_one_slot_time * inst_slot_number;//设置下个周期开始时间
                state = STA_IDLE;
                break;
            }
            while(tx_status == TX_WAIT)                             //等待发送成功，tx_status在发送成功中断内变更状态
            {
                // if(portGetTickCnt() >= (range_time + 50))           //超时没有中断信号，故障，重启
                // {
                //     HAL_NVIC_SystemReset();  //重启
                // }
            }
            tx_status = TX_WAIT;                    //清标志
            valid_report = resp_valid;
            range_status = RANGE_TWR_OK;            //设置TWR成功测距标志，在dw_main.c里判断打包串口输出       
            next_period_time = range_time + inst_one_slot_time * inst_slot_number + tagSleepCorrection_ms;//设置下个周期开始时间
            tagSleepCorrection_ms = 0;
#ifndef L151_DEV
			// if(USE_IMU == 1)
			// {
			// 	//读取IMU数据
			// 	IcmGetRawData(&stAccData[range_nb], &stGyroData[range_nb]);
			// }
#endif
            state = STA_IDLE;
        }
            break;
        
        case STA_IDLE:
#if defined(L151_DEV)
            read_key();
#endif
            if((portGetTickCnt()) >= next_period_time)  //下个周期发送时间到
            {
                dwt_forcetrxoff();
                {
                    for(int i = 0; i < 4; i++){
                        cir_read_flag[i] = 0;
                    }
                    state = STA_SEND_POLL;
                }
            }

            break;
        
        default:
            break;
    }

}



/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_ok_cb()
 *
 * @brief Callback to process RX good frame events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_ok_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_OK;
    if (cb_data->datalength <= FRAME_LEN_MAX)
    {
        dwt_readrxdata(rx_buffer, cb_data->datalength, 0);
    }

    UNUSED(cb_data);

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_to_cb()
 *
 * @brief Callback to process RX timeout events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_to_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_TIMEOUT;
    UNUSED(cb_data);

}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_rx_err_cb()
 *
 * @brief Callback to process RX error events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_rx_err_cb(const dwt_cb_data_t *cb_data)
{
    rx_status = RX_ERROR;
    UNUSED(cb_data);
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn tag_tx_conf_cb()
 *
 * @brief Callback to process TX confirmation events
 *
 * @param  cb_data  callback data
 *
 * @return  none
 */
void tag_tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    tx_status = TX_OK;
    UNUSED(cb_data);
}
