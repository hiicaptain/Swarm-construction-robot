/**
  ******************************************************************************
  * 文件名        ：soft_i2c.h
	* 创建时间      ：2019.4.10
	* 作者          ：谢胜
	*-----------------------------------------------------------------------------
	* 最近修改时间  ：2019.11.24
	* 修改人        ：刘文熠
  ******************************************************************************
	* 1.本代码基于STM32F427IIH6开发，编程环境位Keil 5，基于FreeRTOS进行开发
	* 2.本代码只适用于Robomaster机器人
	* 3.本代码未经允许禁止私自转发、禁止私自使用
	* 4.本代码包含大量中文注释，请以ANSI编码格式打开
	* 5.本代码最终解释权归哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT所有
	* 
	* Copyright (c) 哈尔滨工业大学（深圳）南工骁鹰战队Critical HIT 版权所有
	******************************************************************************
  */

#ifndef SOFT_I2C_H
#define SOFT_I2C_H

#include "stm32f4xx.h"

extern GPIO_TypeDef *IIC_GPIO;
extern uint16_t SDA;
extern uint16_t SCL;

int IIC_ReadData(uint8_t dev_addr, uint8_t reg_addr, uint8_t * pdata, uint8_t count);
int IIC_WriteData(uint8_t dev_addr,uint8_t reg_addr,uint8_t data);

#endif
