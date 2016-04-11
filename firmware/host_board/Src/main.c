/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"

//i2c timeout is in interval of systemtick
//HAL library reads as a 8 bit addresss, bitshifting required!

#define I2C_MUX_ADDR 0x70<<1 //TCA9548A address

//for sensor board
#define I2C_TEMP_SENSOR_ADDR 0x18<<1 //MCP9808
#define I2C_IO_ADDR 0x41<<1 //PCA9536
#define I2C_COLOR_SENSOR_ADDR 0x29<<1 //TCS34725
#define I2C_TIMEOUT 100000

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);

void I2C_Change_Chanel(uint8_t chanel);
void I2C_Flask_Sensor_Config(void);
uint16_t I2C_Read_Temp(void);
void I2C_Enable_Light(void);
void I2C_Disable_Light(void);
void I2C_Measure_Light(uint16_t colors[4]);

volatile uint32_t i = 0;
volatile uint16_t temp;
volatile HAL_StatusTypeDef debug; 


int main(void)
{

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  HAL_MspInit();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  
  MX_I2C1_Init();
  MX_USB_DEVICE_Init();


  //init board 
  //enable power outputs
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_SET);



  I2C_Change_Chanel(0);
  I2C_Flask_Sensor_Config();
  while (1)
  {
  
	i++;
  HAL_Delay(500);
  //uint8_t HiMsg[]="hello\r\n";
  //CDC_Transmit_FS(HiMsg,strlen(HiMsg));
  //temp = I2C_Read_Temp();
  //I2C_Change_Chanel(0);
  uint8_t command_buffer[2];

  //configure I2C IO LED Pin (Pin 1) for output
  command_buffer[0] = 0x03; //write command register
  command_buffer[1] = 0xFE; //set pin 0 as an output
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_IO_ADDR, command_buffer, 2, I2C_TIMEOUT);
  HAL_Delay(500);
  I2C_Enable_Light();
  HAL_Delay(500);
  I2C_Disable_Light();



  }


}

void I2C_Change_Chanel(uint8_t chanel){
  //address I2C Mux and change I2C chanel
  uint8_t tx_data[1];
  tx_data[0] = (uint8_t) 1 << chanel; //select chanel to be active

  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_MUX_ADDR, tx_data, 1, I2C_TIMEOUT);

}

void I2C_Flask_Sensor_Config(void){
  
  uint8_t command_buffer[2];

  //configure I2C IO LED Pin (Pin 1) for output
  command_buffer[0] = 0x03; //write command register
  command_buffer[1] = 0xFE; //set pin 0 as an output
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_IO_ADDR, command_buffer, 2, I2C_TIMEOUT);

  //configure light sensor
  command_buffer[0] = 0x80; //address for control register
  command_buffer[1] = 0x01; //power on
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_COLOR_SENSOR_ADDR, command_buffer, 2, I2C_TIMEOUT);
  HAL_Delay(3); //delay for 3 ms
  command_buffer[1] = 0x03; //enable analog
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_COLOR_SENSOR_ADDR, command_buffer, 2, I2C_TIMEOUT);
  command_buffer[0] = 0x81; //address for integration time register
  command_buffer[1] = 0x00; //maximum integration time
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_COLOR_SENSOR_ADDR, command_buffer, 2, I2C_TIMEOUT);
  command_buffer[0] = 0x8F; //address for RGBC Gain Control
  command_buffer[1] = 0x02; //16x gain
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_COLOR_SENSOR_ADDR, command_buffer, 2, I2C_TIMEOUT);
}

uint16_t I2C_Read_Temp(void){
  uint8_t rx_data[2];
  uint8_t tx_data[1];
  tx_data[0] = 0x05; //Ta address
  //write address then read back temp data
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_TEMP_SENSOR_ADDR, tx_data, 1, I2C_TIMEOUT);

  debug = HAL_I2C_Master_Receive(&hi2c1, I2C_TEMP_SENSOR_ADDR, rx_data, 2, I2C_TIMEOUT);
  return (rx_data[1] | (rx_data[0]<<8));
}

void I2C_Enable_Light(void){
  //pin 0 on I2C MUX
  uint8_t command_buffer[2];
  command_buffer[0] = 0x01; //output port register
  command_buffer[1] = 0x01; //turn on pin 1
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_IO_ADDR, command_buffer, 2, I2C_TIMEOUT);
}

void I2C_Disable_Light(void){
  //pin 0 on I2C MUX
  uint8_t command_buffer[2];
  command_buffer[0] = 0x01; //output port register
  command_buffer[1] = 0x00; //turn off pin 1
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_IO_ADDR, command_buffer, 2, I2C_TIMEOUT);
}

void I2C_Measure_Light(uint16_t colors[4]){
  //yes, clear is a color
  //return in order RGBC
  uint8_t command_buffer[1];
  uint8_t read_buffer[8];

  command_buffer[0] = 0xB4; //consecutive read out, base address of clear
  debug = HAL_I2C_Master_Transmit(&hi2c1, I2C_COLOR_SENSOR_ADDR, command_buffer, 2, I2C_TIMEOUT);
  
  debug = HAL_I2C_Master_Receive(&hi2c1, I2C_COLOR_SENSOR_ADDR, read_buffer, 8, I2C_TIMEOUT);

  //transpose results
  colors[3] = read_buffer[0] | (read_buffer[1]<<8); //clear
  colors[0] = read_buffer[2] | (read_buffer[3]<<8); //red
  colors[1] = read_buffer[4] | (read_buffer[5]<<8); //green
  colors[2] = read_buffer[6] | (read_buffer[7]<<8); //blue

}

/* System Clock Configuration*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI48;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
  HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* I2C1 init function */
void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;

  HAL_I2C_MspInit(&hi2c1);

  HAL_I2C_Init(&hi2c1);

    /**Configure Analogue filter 
    */
  HAL_I2CEx_AnalogFilter_Config(&hi2c1, I2C_ANALOGFILTER_ENABLED);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA2 PA4 PA6 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_2|GPIO_PIN_4|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA3 PA5 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_3|GPIO_PIN_5|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
