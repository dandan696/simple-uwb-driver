#include "instance.h"

uint8_t switch8 = 0b00000001;                           //拨码开关键值
uint8_t instance_mode = ANCHOR;                         //设备运行角色
uint8_t dev_id;                                         //设备ID
uint8_t anc_id;                                         //如当前角色是基站，则表示当前基站ID
uint8_t tag_id;                                         //如当前角色是标签，则表示当前标签ID
uint8_t state = STA_IDLE;                               //状态机状态控制
int32_t distance_report[MAX_AHCHOR_NUMBER];             //基站测距值数组，用于打包输出
uint32_t range_time;                                    //测距产生时间，串口打包发送
uint64_t range_global_time;                             //测距产生时间
uint8_t valid_report = 0x00;                            //基站数据有效标志
uint8_t frame_seq_nb = 0;                               //每帧数据增加1
uint8_t range_nb = 0;                                   //每次range增加1(poll resp1~4 fianl维护一套range_nb)
uint8_t recv_tag_id;                                    //如当前角色是基站，则表示当前基站收到标签发送过来数据的标签ID
uint8_t recv_anc_id;                                    //如当前角色是标签，则表示当前标签收到基站发送过来数据的基站ID
uint8_t range_status = RANGE_NULL;                      //测距成功标志位，用于打包输出
float rx_power;                                         //接收RSSI
static char lcd_data[10];                               //OLED显示数据
uint16_t inst_slot_number;                              //系统内最大标签容量
uint8_t inst_dataRate;                                  //通信速率，用于根据当前110K还是6.8M确定数据超时等通信过程相关参数
uint8_t inst_ch;                                        //信道号Channel number
uint8_t inst_one_slot_time;                             //一个slot的时间，根据通信速率不同而不同，单位ms
uint32_t inst_final_rx_timeout;                         //基站final接收超时时间，根据通信速率不同而不同，单位us
uint32_t inst_resp_rx_timeout;                          //标签发送poll后接收resp超时时间，根据通信速率不同而不同，单位us
uint32_t inst_init_rx_timeout;                          //标签发送blink后接收init超时时间，根据通信速率不同而不同，单位us
uint64_t inst_poll2final_time;                          //单TWR周期poll起始到final结束的总时间
uint32_t inst_data_interval;                            //相邻两条数据的间隔，如poll和第一个resp的间隔，resp1和resp2的间隔，根据通信速率不同而不同，单位us
uint16 ant_dly = ANT_DLY;                               //天线延时
uint32 tx_power;                                        //发射增益代码
uint8_t UART_RX_BUF[200];                               //串口接收BUF
uint32_t uart_rx_len;                                   //串口接收数据长度
vec3d anchorArray[MAX_AHCHOR_NUMBER];                   //基站坐标，用于标签解算自身位置
double distance_now_m;                                  //基站计算本周期测距结果，单位米
int32 distance_offset_cm;                               //距离校准，单位cm
static uint8 Rr;//IMU和测距数据时间对齐
volatile uwb_st myUWB = {0};
uint8_t cir_buffer[8500];
int cir_buffer_start_idx = sizeof(uwb_st) + 4;
const int cir_size = 1016; // 单个基站的cir数据个数
const int cir_buffer_size = 8128; // cir_size * sizeof(uint16_t) * 4

#if defined (DEBUG)
uint64_t tag_poll_send;                                 //标签发送poll的时间（标签端TWR测距起始时间）
uint64_t tag_resp_recv_s[8] = {0};                      //标签接收多个resp开启接收机的时间，注意不是收到的时间，是开启接收的时间
uint64_t tag_resp_recv_r[8] = {0};                      //标签接收多个resp实际接收到的时间
uint64_t tag_final_send;                                //标签发送final的时间

uint64_t anc_poll_recvd;                                //基站接收到poll的时间（基站端TWR测距起始时间）
uint64_t anc_resp_sorr[8] = {0};                        //基站发送resp的时间或接收其他基站的resp开启接收机的时间
uint64_t anc_final_recv;                                //基站接收final开启接收机的时间，注意不是收到的时间，是开启接收的时间
uint64_t anc_final_recvd;                               //基站接收到final的时间

int debug_dly_time = 0;                                 //延时测试
uint8_t UART_TIME_DEBUG[256];                           //串口打包发送
uint32_t debug_time;
#endif


/*******************函数声明***************************/
void parse_uart(uint8_t* data);
void read_anc_coord(void);
void print_config(void);

#if defined (ULM1) || defined (LD150) || defined (LD_PA)
void DW1000_init(void)
{
    uint8_t e2prom_data_read[EEP_UNIT_SIZE] = {0};
    uint8_t e2prom_data_write[EEP_UNIT_SIZE] = {0};
    static dwt_config_t config1 = {         //6.8M
        .chan = 2,                          /* Channel number. */
        .prf = DWT_PRF_64M,                 /* Pulse repetition frequency. */
        .txPreambLength = DWT_PLEN_128,     /* Preamble length. Used in TX only. */
        .rxPAC = DWT_PAC8,                  /* Preamble acquisition chunk size. Used in RX only. */
        .txCode = 9,                        /* TX preamble code. Used in TX only. */
        .rxCode = 9,                        /* RX preamble code. Used in RX only. */
        .nsSFD = 1,                         /* 0 to use standard SFD, 1 to use non-standard SFD. */
        .dataRate = DWT_BR_6M8,             /* Data rate. */
        .phrMode = DWT_PHRMODE_STD,         /* PHY header mode. */
        .sfdTO = (129 + 8 - 8)              /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    };

    static dwt_config_t config2 = {         //110K
        .chan = 2,                          /* Channel number. */
        .prf = DWT_PRF_64M,                 /* Pulse repetition frequency. */
        .txPreambLength = DWT_PLEN_1024,    /* Preamble length. Used in TX only. */
        .rxPAC = DWT_PAC32,                 /* Preamble acquisition chunk size. Used in RX only. */
        .txCode = 9,                        /* TX preamble code. Used in TX only. */
        .rxCode = 9,                        /* RX preamble code. Used in RX only. */
        .nsSFD = 1,                         /* 0 to use standard SFD, 1 to use non-standard SFD. */
        .dataRate = DWT_BR_110K,            /* Data rate. */
        .phrMode = DWT_PHRMODE_STD,         /* PHY header mode. */
        .sfdTO = (1025 + 64 - 32)           /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */
    };

    static dwt_txconfig_t txconfig_options = {
        .PGdly = 0XB5,            /* PG delay */
        .power = TX_POWER         /* TX power */
    };

    reset_DW1000(); /* Target specific drive of RSTn line into DW1000 low for a period. */
    port_set_dw1000_slowrate();

    if(DWT_DEVICE_ID != dwt_readdevid()) //如读取失败，先执行唤醒
    {
    	port_wakeup_IC(); //使用SPI-NS管脚唤醒DW1000
        dwt_softreset();  //软件复位
    }
    reset_DW1000();       //复位

    if (dwt_initialise(DWT_LOADUCODE) == DWT_ERROR)
    {
        led_on(LED_ALL);//DW1000初始化错误
        if(USE_OLED == 1)
        {
            lcd_display(1,2, "  ERROR  ");
            lcd_display(1,3, "INIT FAIL");
        }
        while (1);
    }
    port_set_dw1000_fastrate();


    /* 1号：IMU标签 on=输出IMU数据， off=输出正常mc数据
     * 2号：通信特征  ULM1_2_Off=4tags|110K|100ms|CH2   ULM1_2_On=1tag|6.8M|10ms|CH2
     * 3号：外部耗电开关，用于加大设备功耗，避免充电宝自动关闭，on=开启，off=关闭
     * 4号：角色控制，on=基站，off=标签
     * 5，6，7号：设备ID，000=0，001=1，010=2 ....
     * 8号：硬件测距卡尔曼滤波开关，on=开启，off=关闭（默认on）
     */
#if defined(L151_DEV)
    read_l151_eeprom(L151_SW8_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
    if(e2prom_data_read[0] == 0xAA)
    {
        switch8 = e2prom_data_read[1];         
    }
#else
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        memset(e2prom_data_read, EEP_UNIT_SIZE, 0);
        E2prom_Read(SW8_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA)
        {
            switch8 = e2prom_data_read[1];         
        }
        else
        {
            switch8 = switch_is_on(SW_8)   //读取拨码开关键值
                    | switch_is_on(SW_7) << 1    
                    | switch_is_on(SW_6) << 2
                    | switch_is_on(SW_5) << 3
                    | switch_is_on(SW_4) << 4
                    | switch_is_on(SW_3) << 5
                    | switch_is_on(SW_2) << 6
                    | switch_is_on(SW_1) << 7;
        }
    }
#endif

    if(switch8 & SWS1_SHF_MODE) //ULM1_2_On=1tag|6.8M|10ms|CH5
    {
        memset(e2prom_data_read, EEP_UNIT_SIZE, 0);
#if defined(L151_DEV)
        read_l151_eeprom(L151_SLOT_2ON_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
#else
        E2prom_Read(SLOT_2ON_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
#endif
        if(e2prom_data_read[0] == 0xAA)
        {
            inst_slot_number = e2prom_data_read[1];         
        }
        else
        {
            inst_slot_number = MAX_TAG_NUMBER_SW2_ON;
        }
        dwt_configure(&config1);//设置射频通信参数
        inst_dataRate = config1.dataRate;
        inst_ch = config1.chan;
    }
    else //ULM1_2_Off=4tags|110K|100ms|CH2
    {

        memset(e2prom_data_read, EEP_UNIT_SIZE, 0);
#if defined(L151_DEV)
        read_l151_eeprom(L151_SLOT_2OFF_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
#else
        E2prom_Read(SLOT_2OFF_ADDR, &e2prom_data_read[0], EEP_UNIT_SIZE);
#endif            
        if(e2prom_data_read[0] == 0xAA)
        {
            inst_slot_number = e2prom_data_read[1];         
        }
        else
        {
            inst_slot_number = MAX_TAG_NUMBER_SW2_OFF;
        }
        dwt_configure(&config2);//设置射频通信参数
        inst_dataRate = config2.dataRate;
        inst_ch = config2.chan;
    }

    if(inst_dataRate == DWT_BR_6M8)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_6P8M;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_6P8M;
        inst_resp_rx_timeout = RESP_RX_TIMEOUT_6P8M;
        inst_init_rx_timeout = INIT_RX_TIMEOUT_6P8M;
        inst_data_interval = DATA_INTERVAL_TIME_6P8M;
        inst_poll2final_time = ((MAX_AHCHOR_NUMBER + 1) * inst_data_interval * UUS_TO_DWT_TIME);
    }
    else if(inst_dataRate == DWT_BR_110K)
    {
        inst_one_slot_time = ONE_SLOT_TIME_MS_110K;
        inst_final_rx_timeout = FINAL_RX_TIMEOUT_110K;
        inst_resp_rx_timeout = RESP_RX_TIMEOUT_110K;
        inst_init_rx_timeout = INIT_RX_TIMEOUT_110K;
        inst_data_interval = DATA_INTERVAL_TIME_110K;
#if (MAX_AHCHOR_NUMBER == 4)
        inst_poll2final_time = ((MAX_AHCHOR_NUMBER + 1) * inst_data_interval * UUS_TO_DWT_TIME);
#elif (MAX_AHCHOR_NUMBER == 8)
        inst_poll2final_time = ((MAX_AHCHOR_NUMBER + 2) * inst_data_interval * UUS_TO_DWT_TIME);  //8基站final消息长，多预留final消息时间
#endif
    }
#if defined(L151_DEV)
           
#else
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        memset(e2prom_data_read, EEP_UNIT_SIZE, 0);
        E2prom_Read(TX_PWR_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA) //固定AA
        {
            txconfig_options.power  = (uint32)e2prom_data_read[1] << 24;
            txconfig_options.power += (uint32)e2prom_data_read[2] << 16;
            txconfig_options.power += (uint32)e2prom_data_read[3] << 8;
            txconfig_options.power += (uint32)e2prom_data_read[4];
        }
        else    //未配置，写入默认值
        {
            txconfig_options.power = TX_POWER;
            memset(e2prom_data_write, EEP_UNIT_SIZE, 0);
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = txconfig_options.power >> 24;
            e2prom_data_write[2] = txconfig_options.power >> 16;
            e2prom_data_write[3] = txconfig_options.power >> 8;
            e2prom_data_write[4] = txconfig_options.power;
            E2prom_Write(TX_PWR_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
        }
    }
    else    //未板载EEPROM，按define值设置
    {
        txconfig_options.power = TX_POWER;
    }
#endif
    tx_power = txconfig_options.power;
    dwt_configuretxrf(&txconfig_options);//设置发射功率和pg值

#if defined(L151_DEV)
           
#else
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        memset(e2prom_data_read, EEP_UNIT_SIZE, 0);
        E2prom_Read(ANT_DLY_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA) //固定AA
        {
            ant_dly = (uint16)e2prom_data_read[1] << 8 | (uint16)e2prom_data_read[2];
        }
        else    //未配置，写入默认值
        {
            ant_dly = ANT_DLY;
            memset(e2prom_data_write, EEP_UNIT_SIZE, 0);
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = ant_dly >> 8;
            e2prom_data_write[2] = (uint8)ant_dly;
            E2prom_Write(ANT_DLY_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
        }
    }
    else    //未板载EEPROM，按define值设置
    {
        ant_dly = ANT_DLY;
    }
#endif
    dwt_setrxantennadelay(ant_dly);//设置天线延时
    dwt_settxantennadelay(ant_dly);

    dwt_setpanid(PAN_ID);//设置PAN ID 组号
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_ACK_EN);  //设置帧过滤模式开启

    dwt_setlnapamode(1, 1);//设置外置PA和LNA控制开启
    dwt_setleds(DWT_LEDS_ENABLE | DWT_LEDS_INIT_BLINK);//设置DW3000控制的收发指示灯开启，低功耗时可注释掉

    if(switch8 & SWS1_ROLE_MODE)//第4拨码开关on=anchor
    {
        instance_mode = ANCHOR; //当前角色控制为基站
    }
    else
    {
        instance_mode = TAG; //当前角色控制为标签
    }

    if(USE_OLED == 1)//显示版本号，系统最大基站个数，最大标签容量，如“4A10T”表示最大4基站10标签
    {
        if(inst_slot_number < 10)
        {
            sprintf(lcd_data, "%s  %dA%dT", SOFTWARE_VER, MAX_AHCHOR_NUMBER, inst_slot_number);
        }
        else
        {          
            sprintf(lcd_data, "%s %dA%dT", SOFTWARE_VER, MAX_AHCHOR_NUMBER, inst_slot_number);
        }
        lcd_display(1, 1, lcd_data);

        sprintf(lcd_data, "%s", (inst_dataRate == DWT_BR_110K)? "110K" : "6.8M");
        lcd_display(1, 3, lcd_data);
        sprintf(lcd_data,"CH%d", inst_ch);
        lcd_display(7, 3, lcd_data);
        if(instance_mode == ANCHOR)
        {
            sprintf(lcd_data, "%dms", inst_slot_number * inst_one_slot_time);
            lcd_display(1, 2, lcd_data);
        }

        if(switch8 & SWS1_KAM_MODE)//第8拨码开关=on,开启卡尔曼滤波
        {
            lcd_display(9, 4, "K");//屏幕右下脚显示K，表示开启
        }
#ifndef L151_DEV
        if(switch8 & SWS1_HPR_MODE)//第3拨码开关on=打开外部耗电开关
        {
            expr_on();
            lcd_display(8, 4, "P");//屏幕右下脚显示K，表示开启
        }
#endif

    }

    //设置设备ID
#if defined(L151_DEV)
    if(*(__IO uint8_t*)(L151_DEV_ID_ADDR) == 0xAA)
    {
        dev_id = *(__IO uint8_t*)(L151_DEV_ID_ADDR + 1);
    }
    else
    {
        dev_id = 0;
    }
#else
    if(USE_EXT_EEPROM == 1) //板载EEPROM
    {
        memset(e2prom_data_read, EEP_UNIT_SIZE, 0);
        E2prom_Read(DEV_ID_ADDR, e2prom_data_read, EEP_UNIT_SIZE);
        if(e2prom_data_read[0] == 0xAA) //固定AA
        {
            dev_id = e2prom_data_read[1];
        }
        else
        {
            //读取拨码开关设备ID
            dev_id = ((switch8 & SWS1_A1A_MODE) + (switch8 & SWS1_A2A_MODE) + (switch8 & SWS1_A3A_MODE)) >> 1;
        }
    }
    else
    {
        //读取拨码开关设备ID
        dev_id = ((switch8 & SWS1_A1A_MODE) + (switch8 & SWS1_A2A_MODE) + (switch8 & SWS1_A3A_MODE)) >> 1;
    }
#endif

    //设置中断标志
    dwt_setinterrupt(DWT_INT_TFRS | DWT_INT_RFCG | (DWT_INT_ARFE | DWT_INT_RFSL | DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_RXPTO), 1);
    
    if(instance_mode == ANCHOR)
    {
        //设置基站的中断回调函数
#ifndef L151_DEV
        dwt_setcallbacks(&anc_tx_conf_cb, &anc_rx_ok_cb, &anc_rx_to_cb, &anc_rx_err_cb);
#endif
    }
    else
    {   //设置标签的中断回调函数
        dwt_setcallbacks(&tag_tx_conf_cb, &tag_rx_ok_cb, &tag_rx_to_cb, &tag_rx_err_cb);
    }
    
    //按角色初始化设备短地址，设置状态机初始状态
    if(instance_mode == ANCHOR)
    {
        /* 设备短地址为2个字节，为了区分A0和T0短地址，基站的最高位为1
         * 如A1短地址=0x8001，T1短地址=0x0001
         */
        uint16_t anc_short_add = 0x8000 | dev_id;
        dwt_setaddress16(anc_short_add);
        anc_id = dev_id;
        if(USE_OLED == 1)
        {
            sprintf(lcd_data,"Anc:%d",dev_id);
            lcd_display(1, 4, lcd_data);
        }
        state = STA_INIT_POLL_BLINK;

#if defined (LD150) || defined (LD_PA)
        led_on(LED1); 
#endif
    }
    else //tag
    {
        dwt_setaddress16(dev_id);
        tag_id = dev_id;
        if(USE_OLED == 1)
        {
            sprintf(lcd_data,"Tag:%d",dev_id);
            lcd_display(1, 4, lcd_data);
        }
        state = STA_SEND_POLL;
    }
}
#endif




void dw_main(void)
{
    // uint8_t UART_TX_DATA[512], UART_TX_DEBUG[128];//串口数据输出
    int len,len2=0;
    led_off(LED_ALL);

    DW1000_init();

    MX_IWDG_Init();

    if(instance_mode == TAG)
    {
        read_anc_coord();//读取配置的基站坐标，用于标签解算输出自身坐标
    }

    //打印系统参数信息
    //print_config();
/*        while(1)//测距功能实现，按角色执行基站状态机或标签状态机
    {
        HAL_IWDG_Refresh(&hiwdg); //喂狗
        range_nb++;
        if(range_nb == 0) Rr = 255;
                    else Rr = range_nb - 1;
        IcmGetRawData(&stAccData[range_nb], &stGyroData[range_nb]);
        range_global_time = GetGlobal_Us(); //获取测距时间，用于dw_main.c中串口打包

            UART_TX_DATA[0] = 0xee;
            for(int h=0; h<11;h++)
            {
             UART_TX_DATA[9+h*5] = 0xdd;
            }
            uint8_t byteArray1[8];
            uint8_t byteArray2[4];
            int64ToByteArray(range_global_time,byteArray1);
            memcpy(&UART_TX_DATA[1], byteArray1, 8);
            for(int h=0; h<4; h++)
            {
             
             int32ToByteArray(distance_report[h],byteArray2);
             memcpy(&UART_TX_DATA[10 + 5*h], byteArray2, 4);
            }
            floatToByteArray(stAccData[range_nb].x,byteArray2);
            memcpy(&UART_TX_DATA[30], byteArray2, 4);
            floatToByteArray(stAccData[range_nb].y,byteArray2);   
            memcpy(&UART_TX_DATA[35], byteArray2, 4);
            floatToByteArray(stAccData[range_nb].z,byteArray2);
            memcpy(&UART_TX_DATA[40], byteArray2, 4);
            floatToByteArray(stGyroData[range_nb].x,byteArray2);  
            memcpy(&UART_TX_DATA[45], byteArray2, 4);
            floatToByteArray(stGyroData[range_nb].y,byteArray2);  
            memcpy(&UART_TX_DATA[50], byteArray2, 4);
            floatToByteArray(stGyroData[Rr].z,byteArray2);
            memcpy(&UART_TX_DATA[55], byteArray2, 4);

           
            HAL_UART_Transmit_DMA(&UART_PORT, &UART_TX_DATA[0], 60);//串口发送接收到的数据

    }
*/

    while(1)//测距功能实现，按角色执行基站状态机或标签状态机
    {
        HAL_IWDG_Refresh(&hiwdg); //喂狗
        if(instance_mode == ANCHOR)
        {
#ifndef L151_DEV
            anchor_app();
#endif
        }
        else
        {
            tag_app();
        }

        if(range_status == RANGE_TWR_OK) //TWR测距有效，进行数据滤波和打包输出、屏显
        {
            range_status = RANGE_NULL;//清空标志位
            if(switch8 & SWS1_KAM_MODE)//第8拨码开关=on,开启卡尔曼滤波
            {

                for(int i=0; i<MAX_AHCHOR_NUMBER; i++)
                {
                    // if(distance_report[i] > 0) //数据有效，进行卡尔曼滤波计算
                    // {
                    //     distance_report[i] = kalman_filter(distance_report[i],i,(instance_mode == TAG)?tag_id:recv_tag_id);
                    // }
                }


            }
            //串口数据打包发送
            len2=0;
            static uint16_t fn = 0;         //串口数据流水号，每帧数据+1

#if (MAX_AHCHOR_NUMBER == 8)
            len = sprintf((char*)&UART_TX_DATA[0], "mc 00 %08x %08x %08x %08x %08x %08x %08x %08x %04x %02x %08x %c%d:%d %04x\r\n",
                        distance_report[0], distance_report[1], distance_report[2], distance_report[3], 
                        distance_report[4], distance_report[5], distance_report[6], distance_report[7], 
                        fn++, range_nb, range_time, (instance_mode == TAG)?'t':'a', (instance_mode == TAG)?tag_id:recv_tag_id, (instance_mode == TAG)?0:anc_id, (int)(-(rx_power*100)));
#elif (MAX_AHCHOR_NUMBER == 4)

            // len = sprintf((char*)&UART_TX_DATA[0], "mc 00 %08x %08x %08x %08x %04x %02x %08x %c%d:%d %04x\r\n",
            //             distance_report[0], distance_report[1], distance_report[2], distance_report[3], 
            //             fn++, range_nb, range_time, (instance_mode == TAG)?'t':'a', (instance_mode == TAG)?tag_id:recv_tag_id, (instance_mode == TAG)?0:anc_id, (int)(-(rx_power*100)));
            len = 1;
#endif
    
            char Location_char[30]="LO=[not calculated]";
            if((instance_mode == TAG) && ((inst_slot_number > 5) || (inst_dataRate != DWT_BR_6M8)))
            {
                int result = 0; 
                vec3d report;
                result = GetLocation(&report, &anchorArray[0], &distance_report[0]);
                
                if(result > 0)
                {
                    sprintf(Location_char,"LO=[%.2f,%.2f,%.2f]",report.x,report.y,report.z);
                }
                else
                {
                    sprintf(Location_char,"LO=[no solution]");
                }
            }

            char distance_report_char[MAX_AHCHOR_NUMBER][20];
            for(int i = 0; i<MAX_AHCHOR_NUMBER; i++)
            {
                if(distance_report[i] >= 0)
                {
                    sprintf(distance_report_char[i], "%.2f", (float)distance_report[i]/1000.0);
                }
                else
                {
                    sprintf(distance_report_char[i], "%s", "NULL");
                }
            }
            if(instance_mode == TAG)
            {
#if (MAX_AHCHOR_NUMBER == 8)
#ifndef L151_DEV
                if((USE_IMU == 1) && (switch8 & SWS1_IMU_MODE)) //1号拨码on
                {
                    static uint8 Rr;//IMU和测距数据时间对齐
                    if(range_nb == 0) Rr = 255;
                    else Rr = range_nb - 1;
                    len = sprintf((char*)&UART_TX_DATA[0], "mi,%.3f,%s,%s,%s,%s,%s,%s,%s,%s,%f,%f,%f,%f,%f,%f,T%d\r\n",
																					(float)range_time/1000, 
																					distance_report_char[0], distance_report_char[1], distance_report_char[2], distance_report_char[3], 
																					distance_report_char[4], distance_report_char[5], distance_report_char[6], distance_report_char[7], 
																					stAccData[Rr].x, stAccData[Rr].y, stAccData[Rr].z, 
																					stGyroData[Rr].x, stGyroData[Rr].y, stGyroData[Rr].z, 
																					tag_id);
                    
                    len2 = 0;
                }
                else 
#endif                
                if((inst_slot_number > 1) || (inst_dataRate != DWT_BR_6M8))
                {
                    len2 = sprintf((char*)&UART_TX_DEBUG[0], "$%sT%d,%s,%s,%s,%s,%s,%s,%s,%s,%s\r\n", 
                                        (switch8 & SWS1_KAM_MODE) ? "K":"NK", tag_id, 
                                        distance_report_char[0], distance_report_char[1], distance_report_char[2], distance_report_char[3], 
                                        distance_report_char[4], distance_report_char[5], distance_report_char[6], distance_report_char[7], 
                                        Location_char);
                }
#elif (MAX_AHCHOR_NUMBER == 4) 

#ifndef L151_DEV  
                if((USE_IMU == 1) && (switch8 & SWS1_IMU_MODE)) //1号拨码on
                {
                    if(range_nb == 0) Rr = 255;
                    else Rr = range_nb - 1;
                    // len = sprintf((char*)&UART_TX_DATA[0], "mi,%.3f,%s,%s,%s,%s,%f,%f,%f,%f,%f,%f,T%d\r\n",
                    //         (float)range_time/1000,distance_report_char[0], distance_report_char[1], distance_report_char[2], distance_report_char[3], stAccData[Rr].x, stAccData[Rr].y, stAccData[Rr].z, stGyroData[Rr].x, stGyroData[Rr].y, stGyroData[Rr].z, tag_id);
                    len = 1;
                    len2 = 0;
                }
                else 
#endif
                if((inst_slot_number > 1) || (inst_dataRate != DWT_BR_6M8))
                {
                    // len2 = sprintf((char*)&UART_TX_DEBUG[0], "$%sT%d,%s,%s,%s,%s,%s\r\n", 
                    //                     (switch8 & SWS1_KAM_MODE) ? "K":"NK", tag_id, 
                    //                     distance_report_char[0], distance_report_char[1], distance_report_char[2], distance_report_char[3], 
                    //                     Location_char);
                    len2 = 0;
                }
#endif
            }
            // strcat((char*)&UART_TX_DATA[0], (char*)&UART_TX_DEBUG[0]);
            len = len + len2;
            
            //标签屏显测距距离数据
            if((USE_OLED == 1) && (instance_mode == TAG) && ((inst_slot_number > 5) || (inst_dataRate != DWT_BR_6M8)))
            {
                static uint64_t printLCDTWRReports = 0;
                static int toggle = 0;
                if(printLCDTWRReports + 2000 <= portGetTickCnt()) //每2S更新一次数据
                {
                    printLCDTWRReports = portGetTickCnt();
                    sprintf((char*)&lcd_data[0], "A%d:%3.2fm ", toggle, (float)distance_report[toggle]/1000.0);
                    lcd_display(1,2, lcd_data);                    
                    toggle++;
                    if(toggle >= MAX_AHCHOR_NUMBER)
                        toggle = 0;
                }
            }
        }
        else if(range_status == RANGE_ERROR) 
        {
            range_status = RANGE_NULL;//清空标志位
            // len = sprintf((char*)&UART_TX_DATA[0], "$RANGE_ERROR,ID=%d,rb=%d,rangetime=%d\r\n", dev_id, range_nb, range_time);
            len = 0;
        }

        if(len > 0)
        {
#if defined (DEBUG)
            int len3 = 0;
            if(instance_mode == TAG)
            {
                // len3 = sprintf((char*)&UART_TIME_DEBUG[0], "%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld\r\n%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld\r\n", tag_poll_send, 
                // tag_resp_recv_s[0], tag_resp_recv_s[1], tag_resp_recv_s[2], tag_resp_recv_s[3], 
                // tag_resp_recv_s[4], tag_resp_recv_s[5], tag_resp_recv_s[6], tag_resp_recv_s[7], 
                // tag_final_send,
                // tag_resp_recv_r[0], tag_resp_recv_r[1], tag_resp_recv_r[2], tag_resp_recv_r[3], 
                // tag_resp_recv_r[4], tag_resp_recv_r[5], tag_resp_recv_r[6], tag_resp_recv_r[7]
                // );
                len3 = sprintf((char*)&UART_TIME_DEBUG[0], "debug_time=%d,range_time=%d\r\n", debug_time, range_time);
            }
            else
            {
                // len3 = sprintf((char*)&UART_TIME_DEBUG[0], "%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,%lld,dt=%d\r\n", anc_poll_recvd, 
                // anc_resp_sorr[0], anc_resp_sorr[1], anc_resp_sorr[2], anc_resp_sorr[3], 
                // anc_resp_sorr[4], anc_resp_sorr[5], anc_resp_sorr[6], anc_resp_sorr[7], 
                // anc_final_recv, debug_dly_time);

                //len3 = sprintf((char*)&UART_TIME_DEBUG[0], "%lld\r\n", anc_final_recvd-anc_poll_recvd);

                len3 = sprintf((char*)&UART_TIME_DEBUG[0], "debug_time=%d,range_time=%d\r\n", debug_time, range_time);
            }

            strcat((char*)&UART_TX_DATA[0], (char*)&UART_TIME_DEBUG[0]);
            len = len + len3;
#endif
           

/*
            UART_TX_DATA[0] = 0xee;
            for(int h=0; h<11;h++)
            {
             UART_TX_DATA[9+h*5] = 0xdd;
            }
            uint8_t byteArray1[8];
            uint8_t byteArray2[4];
            int64ToByteArray(range_global_time,byteArray1);
            memcpy(&UART_TX_DATA[1], byteArray1, 8);
            for(int h=0; h<4; h++)
            {
             
             int32ToByteArray(distance_report[h],byteArray2);
             memcpy(&UART_TX_DATA[10 + 5*h], byteArray2, 4);
            }
            floatToByteArray(stAccData[Rr].x,byteArray2);
            memcpy(&UART_TX_DATA[30], byteArray2, 4);
            floatToByteArray(stAccData[Rr].y,byteArray2);   
            memcpy(&UART_TX_DATA[35], byteArray2, 4);
            floatToByteArray(stAccData[Rr].z,byteArray2);
            memcpy(&UART_TX_DATA[40], byteArray2, 4);
            floatToByteArray(stGyroData[Rr].x,byteArray2);  
            memcpy(&UART_TX_DATA[45], byteArray2, 4);
            floatToByteArray(stGyroData[Rr].y,byteArray2);  
            memcpy(&UART_TX_DATA[50], byteArray2, 4);
            floatToByteArray(stGyroData[Rr].z,byteArray2);
            memcpy(&UART_TX_DATA[55], byteArray2, 4);
*/

            // 读取imu
            myUWB.imu.timestamp = GetGlobal_Us();
            IcmGetS16Data(myUWB.imu.acc_s16, myUWB.imu.gyro_s16);

            // UWB观测值赋值
            myUWB.timestamp = timestamp_send_poll;
            myUWB.poll_tx_ts = poll_tx_ts;                     
            myUWB.final_tx_ts = final_tx_ts;
            for (int i = 0; i < MAX_AHCHOR_NUMBER; i++) {
                myUWB.range[i] = distance_report[i];
                myUWB.timestamp_recv[i] = timestamp_recv_resp[i];
                myUWB.resp_rx_ts[i] = resp_rx_ts[i];
            }      
            len = pack_uwb(cir_buffer, &myUWB); // 打包
            HAL_UART_Transmit_DMA(&UART_PORT, &cir_buffer[0], len);//串口发送接收到的数据
            
            // // 计算CIR校验值
            // uint8_t checksum = 0;
            // for(int i = cir_buffer_start_idx; i < cir_buffer_start_idx + cir_buffer_size; i++){
            //     checksum ^= cir_buffer[i];
            // }
            // cir_buffer[cir_buffer_start_idx + cir_buffer_size] = checksum;
            
            // HAL_UART_Transmit_DMA(&UART_PORT, &cir_buffer[0], cir_buffer_start_idx + cir_buffer_size + 1);//串口发送接收到的数据

            // 置零
            len = 0;
        }

        if(uart_rx_len > 0)//收到串口数据
        {
            if((UART_RX_BUF[0] == '$') && (UART_RX_BUF[uart_rx_len - 1] == '\n'))
            {
                parse_uart(&UART_RX_BUF[0]);//数据解析和指令执行
            }
            uart_rx_len = 0;
            memset(UART_RX_BUF, 0, sizeof(UART_RX_BUF));

        }
        
    }
}

void print_config(void)
{
    int len;
    uint8_t UART_TX_DATA[512];
    len = sprintf((char*)&UART_TX_DATA[0], "\r\n**********************DEVICE CONFIG**********************\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

#if defined (ULM1)
    len = sprintf((char*)&UART_TX_DATA[0], "* model = ULM1\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

#elif defined (LD150)
    len = sprintf((char*)&UART_TX_DATA[0], "* model = LD150\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
#elif defined (LD_PA)
    len = sprintf((char*)&UART_TX_DATA[0], "* model = LD_PA\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
#endif

    len = sprintf((char*)&UART_TX_DATA[0], "* firmware = %s\r\n* role = %s\r\n* addr = %d\r\n* blink = %d\r\n", SOFTWARE_VER, (instance_mode == TAG)?"TAG":"ANCHOR", dev_id, 0);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* max_anc_num = %d\r\n* max_tag_num = %d\r\n", MAX_AHCHOR_NUMBER, inst_slot_number);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);


    len = sprintf((char*)&UART_TX_DATA[0], "* uwb_data_rate = %s\r\n* channel = CH%d\r\n", (inst_dataRate == DWT_BR_110K)? "110K" : "6.8M", inst_ch);

    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* updata_frequency = %dHz\r\n* updata_Period = %dms\r\n* kalmanfilter = %d\r\n", 1000 / (inst_slot_number * inst_one_slot_time), inst_slot_number * inst_one_slot_time, (switch8 & SWS1_KAM_MODE)? 1:0);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* ant_dly = %d\r\n* tx_power = %08lx\r\n", ant_dly, tx_power);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

    len = sprintf((char*)&UART_TX_DATA[0], "* use_ext_eeprom = %d\r\n* use_imu = %d\r\n* use_oled = %d\r\n", USE_EXT_EEPROM, USE_IMU, USE_OLED);
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
    if(instance_mode == TAG)
    {
        len = sprintf((char*)&UART_TX_DATA[0], "* A0(%.2f, %.2f, %.2f) A1(%.2f, %.2f, %.2f)\r\n* A2(%.2f, %.2f, %.2f) A3(%.2f, %.2f, %.2f)\r\n", 
                                                    anchorArray[0].x, anchorArray[0].y, anchorArray[0].z, 
                                                    anchorArray[1].x, anchorArray[1].y, anchorArray[1].z,
                                                    anchorArray[2].x, anchorArray[2].y, anchorArray[2].z,
                                                    anchorArray[3].x, anchorArray[3].y, anchorArray[3].z);
        HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);
    }

    len = sprintf((char*)&UART_TX_DATA[0], "***************************END***************************\r\n");
    HAL_UART_Transmit(&UART_PORT, &UART_TX_DATA[0], len, 1000);

}

void int32ToByteArray(int32_t value, uint8_t byteArray[4]) {
    byteArray[0] = (value >> 24) & 0xFF;
    byteArray[1] = (value >> 16) & 0xFF;
    byteArray[2] = (value >> 8) & 0xFF;
    byteArray[3] = value & 0xFF;
}
void int64ToByteArray(int64_t value, uint8_t byteArray[8]) {
    byteArray[0] = (value >> 56) & 0xFF;
    byteArray[1] = (value >> 48) & 0xFF;
    byteArray[2] = (value >> 40) & 0xFF;
    byteArray[3] = (value >> 32) & 0xFF;
    byteArray[4] = (value >> 24) & 0xFF;
    byteArray[5] = (value >> 16) & 0xFF;
    byteArray[6] = (value >> 8) & 0xFF;
    byteArray[7] = value & 0xFF;
}

void floatToByteArray(float value, uint8_t byteArray[4]) {
    union {
        float input;
        uint8_t output[4];
    } data;
    data.input = value;
    memcpy(byteArray, data.output, sizeof(data.output));
}

float byteArrayToFloat(uint8_t byteArray[4]) {
    union {
        float output;
        uint8_t input[4];
    } data;

    memcpy(data.input, byteArray, sizeof(data.input));
    return data.output;
}

int pack_uwb(uint8_t* data, uwb_st* uwb) {
    int cnt = 0;
    data[cnt++] = 0xaa; // 帧头
    data[cnt++] = 0x55; // 帧头
    data[cnt++] = 0x01; // 类型
    int data_start = cnt;

    for(int i = 0; i < sizeof(uwb_st); i++) {
        data[cnt++] = ((uint8_t*)uwb)[i];
    }
    // 校验和
    uint8_t checksum = 0;
    for (int i = data_start; i < cnt; i++) {
        checksum ^= data[i];
    }
    data[cnt++] = checksum;
    return cnt;
}

void read_anc_coord(void)
{
    char anc_coord_read[COORD_LENGTH];
    char coord_cut_data[MAX_AHCHOR_NUMBER*3+1][10];
#if defined(L151_DEV)
           
#else
    E2prom_Read(ANC_COORD_ADDR, (uint8_t*)anc_coord_read, COORD_LENGTH);
#endif
    if(anc_coord_read[0] == '$')
    {
        char *ptr, *retptr;
        ptr = anc_coord_read;
        uint8_t i = 0;

        while((retptr=strtok(ptr,",")) != NULL)
        {
            strcpy(coord_cut_data[i], retptr);
            ptr = NULL;
            i++;
        }

        anchorArray[0].x = atof(coord_cut_data[1]);
        anchorArray[0].y = atof(coord_cut_data[2]);
        anchorArray[0].z = atof(coord_cut_data[3]);

        anchorArray[1].x = atof(coord_cut_data[4]);
        anchorArray[1].y = atof(coord_cut_data[5]);
        anchorArray[1].z = atof(coord_cut_data[6]);

        anchorArray[2].x = atof(coord_cut_data[7]);
        anchorArray[2].y = atof(coord_cut_data[8]);
        anchorArray[2].z = atof(coord_cut_data[9]);

        anchorArray[3].x = atof(coord_cut_data[10]);
        anchorArray[3].y = atof(coord_cut_data[11]);
        anchorArray[3].z = atof(coord_cut_data[12]);
    }
}


/*
    串口指令集，注意发送指令以$开头，以\r\n结尾
    $rboot            重启
    $rantdly          查询天线延时参数
    $reset            恢复默认参数
    $santdly,16375    设置天线延时参数（10进制）
    $stxpwr,1f1f1f1f  设置发射增益参数（16进制）
    $sanccd,0,0,2,0,3.1,2,3.1,0,2,3.1,3.1,2  设置基站坐标A0.X,A0.Y,A0.Z,A1.X,A1,Y,A1,Z,A2.X,A2,Y,A2,Z,A3.X,A3,Y,A3,Z
    $cali,-30,-169    校准距离和PDOA，第一个参数为距离，单位cm，第二个参数为PDOA单位角度
    $saddr,1          设置标签ID
    $ssw,00010001     设置拨码开关值，设置后实体拨码开关将不起作用，16进制 0001 0001
    $s2onslot,1       设置2号拨码on时的最大标签容量
    $s2offslot,1      设置2号拨码off时的最大标签容量
*/

void parse_uart(uint8_t* data)
{
    uint8_t UART_COMMAND_BUF[30];
    uint8_t len = 0;
    uint8_t e2prom_data_write[EEP_UNIT_SIZE] = {0};
    if(strchr((char*)data, ',') > 0)//带参数指令 如 $santdly,11223
    {
        char *ptr, *retptr;
        ptr = (char*)data;
        retptr = strtok(ptr, ",");//解析数据头

        if(strcmp(retptr, "$santdly") == 0)//设置天线延时参数  $santdly,16375
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            ant_dly = atoi(retptr);
#if defined(L151_DEV)
           
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                e2prom_data_write[0] = 0xAA;
                e2prom_data_write[1] = ant_dly >> 8;
                e2prom_data_write[2] = (uint8)ant_dly;
                E2prom_Write(ANT_DLY_ADDR, e2prom_data_write, EEP_UNIT_SIZE); 
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$stxpwr") == 0)//设置发射增益参数  $stxpwr,1f1f1f1f 
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            sscanf(retptr, "%08lx", &tx_power);
#if defined(L151_DEV)
           
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                e2prom_data_write[0] = 0xAA;
                e2prom_data_write[1] = tx_power >> 24;
                e2prom_data_write[2] = tx_power >> 16;
                e2prom_data_write[3] = tx_power >> 8;
                e2prom_data_write[4] = tx_power;
                E2prom_Write(TX_PWR_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            HAL_NVIC_SystemReset();//重启 
        }

        else if(strcmp(retptr, "$sanccd") == 0) //给标签设置基站坐标，用于标签自己三边定位输出定位结果
        {
            retptr[7] = ',';
            uint8_t zero[COORD_LENGTH] = {0};
#if defined(L151_DEV)
           
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(ANC_COORD_ADDR, (uint8_t*)zero, COORD_LENGTH);
                HAL_Delay(10);
                E2prom_Write(ANC_COORD_ADDR, (uint8_t*)retptr, strlen((char*)retptr));
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$saddr") == 0) //设置标签ID
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = atoi(retptr);
#if defined(L151_DEV)
            write_l151_eeprom(L151_DEV_ID_ADDR, e2prom_data_write, 2);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(DEV_ID_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$ssw") == 0) //设置拨码开关值
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            int sw8 = strtol(retptr, NULL, 2);
            e2prom_data_write[1] = sw8;
#if defined(L151_DEV)
            write_l151_eeprom(L151_SW8_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(SW8_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            //len = sprintf((char*)UART_COMMAND_BUF, "e2prom_data_write=0x%x\r\n", e2prom_data_write[1]);
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$s2onslot") == 0) //设置2号拨码on最大标签容量
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = atoi(retptr);
#if defined(L151_DEV)
            write_l151_eeprom(L151_SLOT_2ON_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(SLOT_2ON_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            //len = sprintf((char*)UART_COMMAND_BUF, "e2prom_data_write=0x%x\r\n", e2prom_data_write[1]);
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }
        else if(strcmp(retptr, "$s2offslot") == 0) //设置2号拨码off最大标签容量
        {
            ptr = NULL;
            retptr = strtok(ptr, ",");
            e2prom_data_write[0] = 0xAA;
            e2prom_data_write[1] = atoi(retptr);
#if defined(L151_DEV)
            write_l151_eeprom(L151_SLOT_2OFF_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
#else
            if(USE_EXT_EEPROM == 1) //板载EEPROM
            {
                E2prom_Write(SLOT_2OFF_ADDR, e2prom_data_write, EEP_UNIT_SIZE);
            }
#endif
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            //len = sprintf((char*)UART_COMMAND_BUF, "e2prom_data_write=0x%x\r\n", e2prom_data_write[1]);
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);            
            HAL_NVIC_SystemReset();//重启
        }

    }
    else //无参数指令 如$rboot
    {
        if(strcmp((char*)data, "$rboot\r\n") == 0)//重启
        {
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            HAL_NVIC_SystemReset();//重启 
        }
        else if(strcmp((char*)data, "$rantdly\r\n") == 0)//查询天线延时参数
        {
            len = sprintf((char*)UART_COMMAND_BUF, "ant_dly = %d\r\n", ant_dly);
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
        }
        else if(strcmp((char*)data, "$reset\r\n") == 0)//恢复默认参数
        {
            len = sprintf((char*)UART_COMMAND_BUF, "$setok\r\n");
            HAL_UART_Transmit(&UART_PORT, &UART_COMMAND_BUF[0], len, 1000);
            uint8_t write_zero[256]={0};
#if defined(L151_DEV)
            write_l151_eeprom(L151_START_ADDR, write_zero, 256);
#else
            E2prom_Write(0, write_zero, 256);
#endif
            HAL_NVIC_SystemReset();//重启 
        }
    }
}

void HAL_UART_IdleCpltCallback(UART_HandleTypeDef *huart)
{
    //HAL_UART_Transmit(&UART_PORT, &UART_RX_BUF[0], strlen((char*)UART_RX_BUF), 1000);
    uart_rx_len = strlen((char*)UART_RX_BUF);
}
