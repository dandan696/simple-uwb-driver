/*! ----------------------------------------------------------------------------
 * @file	port.h
 * @brief	HW specific definitions and functions for portability
 *
 * @attention
 *
 * Copyright 2015 (c) DecaWave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author DecaWave
 */


#ifndef PORT_H_
#define PORT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <string.h>
#include "compiler.h"

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"

#define BUFFLEN 	(1024)   //(4096+128)
#define BUF_SIZE	(64)

typedef uint64_t        uint64;
typedef int64_t         int64;

typedef enum
{
    LED1, 
    LED2, 
    LED3, 
    LED_ALL
} led_t;

extern volatile int32_t sys_time_diff;

#define DECAIRQ_EXTI_IRQn		   (EXTI9_5_IRQn)

#define DW1000_RSTn					DW_RESET_Pin
#define DW1000_RSTn_GPIO	        DW_RESET_GPIO_Port

#define DECAIRQ                     DW_IRQn_Pin
#define DECAIRQ_GPIO                DW_IRQn_GPIO_Port

#define SW_1                        SW1_Pin
#define SW_2                        SW2_Pin
#define SW_3					    SW3_Pin
#define SW_4					    SW4_Pin
#define SW_5					    SW5_Pin
#define SW_6					    SW6_Pin
#define SW_7					    SW7_Pin
#define SW_8					    SW8_Pin
#define SW_GPIO                     SW1_GPIO_Port


#define GPIO_ResetBits(x,y)				    HAL_GPIO_WritePin(x,y, RESET)
#define GPIO_SetBits(x,y)					HAL_GPIO_WritePin(x,y, SET)
#define GPIO_ReadInputDataBit(x,y) 		    HAL_GPIO_ReadPin (x,y)


/* NSS pin is SW controllable */
#define port_SPIx_set_chip_select()			HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_SET)
#define port_SPIx_clear_chip_select()		HAL_GPIO_WritePin(DW_NSS_GPIO_Port, DW_NSS_Pin, GPIO_PIN_RESET)


void Sleep(uint32_t Delay);
unsigned long portGetTickCnt(void);

int switch_is_on(uint16_t GPIOpin);

void port_wakeup_IC(void);
void port_wakeup_IC_fast(void);

void port_set_dw1000_slowrate(void);
void port_set_dw1000_fastrate(void);

void process_dwRSTn_irq(void);
void process_deca_irq(void);

void led_on(led_t led);
void led_off(led_t led);
void led_toggle(led_t led);
void expr_on(void);
void pa_pwr_on(void);
void pa_pwr_off(void);

int  peripherals_init(void);
void spi_peripheral_init(void);
void setup_DW1000RSTnIRQ(int enable);
void reset_DW1000(void);

ITStatus EXTI_GetITEnStatus(uint32_t x);

uint32_t port_GetEXT_IRQStatus(void);
uint32_t port_CheckEXT_IRQ(void);
void port_DisableEXT_IRQ(void);
void port_EnableEXT_IRQ(void);
extern uint32_t		HAL_GetTick(void);
HAL_StatusTypeDef 	flush_report_buff(void);


#ifdef __cplusplus
}
#endif


#endif 

