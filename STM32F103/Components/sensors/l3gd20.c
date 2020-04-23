/**
  ******************************************************************************
  * @file    l3gd20.c
  * @author  MCD Application Team
  * @version V1.1.1
  * @date    27-November-2014
  * @brief   This file provides a set of functions needed to manage the L3GD20,
  *          ST MEMS motion sensor, 3-axis digital output gyroscope.  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2014 STMicroelectronics</center></h2>
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
#include "l3gd20.h"
#include "i2c_handler.h"

/** @addtogroup BSP
  * @{
  */ 

/** @addtogroup Components
  * @{
  */ 

/** @addtogroup L3GD20
  * @{
  */

/** @defgroup L3GD20_Private_TypesDefinitions
  * @{
  */
  
/**
  * @}
  */

/** @defgroup L3GD20_Private_Defines
  * @{
  */

/**
  * @}
  */

/** @defgroup L3GD20_Private_Macros
  * @{
  */

/**
  * @}
  */ 
  
/** @defgroup L3GD20_Private_Variables
  * @{
  */ 
GYRO_DrvTypeDef L3gd20Drv =
{
  L3GD20_Init,
  L3GD20_ReadID,
  L3GD20_RebootCmd,
  L3GD20_INT1InterruptConfig,
  L3GD20_EnableIT,
  L3GD20_DisableIT,
  0,
  0,
  L3GD20_FilterConfig,
  L3GD20_FilterCmd,
  L3GD20_ReadXYZAngRate
};

extern I2C_HandleTypeDef hi2c1;

/**
  * @}
  */

/** @defgroup L3GD20_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/** @defgroup L3GD20_Private_Functions
  * @{
  */
    
static GYRO_DrvTypeDef *GyroscopeDrv;

void GYRO_Init(void)
{  
  uint16_t ctrl = 0x0000;
  GYRO_InitTypeDef         L3GD20_InitStructure;
  GYRO_FilterConfigTypeDef L3GD20_FilterStructure = {0,0};
 
  if((L3gd20Drv.ReadID() == I_AM_L3GD20) || (L3gd20Drv.ReadID() == I_AM_L3GD20_TR))
  {
    /* Initialize the Gyroscope driver structure */
    GyroscopeDrv = &L3gd20Drv;

    /* MEMS configuration ----------------------------------------------------*/
    /* Fill the Gyroscope structure */
    L3GD20_InitStructure.Power_Mode = L3GD20_MODE_ACTIVE;
    L3GD20_InitStructure.Output_DataRate = L3GD20_OUTPUT_DATARATE_1;
    L3GD20_InitStructure.Axes_Enable = L3GD20_AXES_ENABLE;
    L3GD20_InitStructure.Band_Width = L3GD20_BANDWIDTH_4;
    L3GD20_InitStructure.BlockData_Update = L3GD20_BlockDataUpdate_Continous;
    L3GD20_InitStructure.Endianness = L3GD20_BLE_LSB;
    L3GD20_InitStructure.Full_Scale = L3GD20_FULLSCALE_500; 
  
    /* Configure MEMS: data rate, power mode, full scale and axes */
    ctrl = (uint16_t) (L3GD20_InitStructure.Power_Mode | L3GD20_InitStructure.Output_DataRate | \
                      L3GD20_InitStructure.Axes_Enable | L3GD20_InitStructure.Band_Width);
  
    ctrl |= (uint16_t) ((L3GD20_InitStructure.BlockData_Update | L3GD20_InitStructure.Endianness | \
                        L3GD20_InitStructure.Full_Scale) << 8);

    /* Configure the Gyroscope main parameters */
    GyroscopeDrv->Init(ctrl);
  
    L3GD20_FilterStructure.HighPassFilter_Mode_Selection =L3GD20_HPM_NORMAL_MODE_RES;
    L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency = L3GD20_HPFCF_0;
  
    ctrl = (uint8_t) ((L3GD20_FilterStructure.HighPassFilter_Mode_Selection |\
                       L3GD20_FilterStructure.HighPassFilter_CutOff_Frequency));    
  
    /* Configure the Gyroscope main parameters */
    GyroscopeDrv->FilterConfig(ctrl) ;
  
    GyroscopeDrv->FilterCmd(L3GD20_HIGHPASSFILTER_ENABLE);
  
  }
}
    
    
    
    
/**
  * @brief  Set L3GD20 Initialization.
  * @param  L3GD20_InitStruct: pointer to a L3GD20_InitTypeDef structure 
  *         that contains the configuration setting for the L3GD20.
  * @retval None
  */
void L3GD20_Init(uint16_t InitStruct)
{  
  static uint8_t ctrl = 0x00;
  static uint8_t ret;
  
  
  /* Write value to MEMS CTRL_REG1 register */
  ctrl = (uint8_t) InitStruct;
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG1_ADDR, &ctrl, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Write value to MEMS CTRL_REG4 register */  
  ctrl = (uint8_t) (InitStruct >> 8);
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG4_ADDR, &ctrl, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Read ID address of L3GD20
  * @param  None
  * @retval ID name
  */
uint8_t L3GD20_ReadID(void)
{
  uint8_t tmp;
  static uint8_t ret;
  

  /* Read WHO I AM register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_WHO_AM_I_ADDR, &tmp);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Return the ID */
  return (uint8_t)tmp;
}

/**
  * @brief  Reboot memory content of L3GD20
  * @param  None
  * @retval None
  */
void L3GD20_RebootCmd(void)
{
  uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read CTRL_REG5 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG5_ADDR, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Enable or Disable the reboot memory */
  tmpreg |= L3GD20_BOOT_REBOOTMEMORY;
  
  /* Write value to MEMS CTRL_REG5 register */
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG5_ADDR, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Set L3GD20 Interrupt INT1 configuration
  * @param  Int1Config: the configuration setting for the L3GD20 Interrupt.
  * @retval None
  */
void L3GD20_INT1InterruptConfig(uint16_t Int1Config)
{
  uint8_t ctrl_cfr = 0x00, ctrl3 = 0x00;
  static uint8_t ret;
  
  /* Read INT1_CFG register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_INT1_CFG_ADDR, &ctrl_cfr);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Read CTRL_REG3 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG3_ADDR, &ctrl3);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  ctrl_cfr &= 0x80;
  ctrl_cfr |= ((uint8_t) Int1Config >> 8);
  
  ctrl3 &= 0xDF;
  ctrl3 |= ((uint8_t) Int1Config);   
  
  /* Write value to MEMS INT1_CFG register */
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_INT1_CFG_ADDR, &ctrl_cfr, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Write value to MEMS CTRL_REG3 register */
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG3_ADDR, &ctrl3, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Enable INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2   
  * @retval None
  */
void L3GD20_EnableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read CTRL_REG3 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG3_ADDR, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  
  if(IntSel == L3GD20_INT1)
  {
    tmpreg &= 0x7F;	
    tmpreg |= L3GD20_INT1INTERRUPT_ENABLE;
  }
  else if(IntSel == L3GD20_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_ENABLE;
  }
  
  /* Write value to MEMS CTRL_REG3 register */
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG3_ADDR, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Disable  INT1 or INT2 interrupt
  * @param  IntSel: choice of INT1 or INT2 
  *      This parameter can be: 
  *        @arg L3GD20_INT1
  *        @arg L3GD20_INT2   
  * @retval None
  */
void L3GD20_DisableIT(uint8_t IntSel)
{  
  uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read CTRL_REG3 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG3_ADDR, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  if(IntSel == L3GD20_INT1)
  {
    tmpreg &= 0x7F;	
    tmpreg |= L3GD20_INT1INTERRUPT_DISABLE;
  }
  else if(IntSel == L3GD20_INT2)
  {
    tmpreg &= 0xF7;
    tmpreg |= L3GD20_INT2INTERRUPT_DISABLE;
  }
  
  /* Write value to MEMS CTRL_REG3 register */
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG3_ADDR, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  FilterStruct: contains the configuration setting for the L3GD20.        
  * @retval None
  */
void L3GD20_FilterConfig(uint8_t FilterStruct) 
{
  uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read CTRL_REG2 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG2_ADDR, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  tmpreg &= 0xC0;
  
  /* Configure MEMS: mode and cutoff frequency */
  tmpreg |= FilterStruct;
  
  /* Write value to MEMS CTRL_REG2 register */
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG2_ADDR, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: L3GD20_HIGHPASSFILTER_DISABLE 
  *         @arg: L3GD20_HIGHPASSFILTER_ENABLE          
  * @retval None
  */
void L3GD20_FilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read CTRL_REG5 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG5_ADDR, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  tmpreg &= 0xEF;
  
  tmpreg |= HighPassFilterState;
  
  /* Write value to MEMS CTRL_REG5 register */
  ret = I2Cx_WriteData(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG5_ADDR, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
}

/**
  * @brief  Get status for L3GD20 data
  * @param  None         
  * @retval Data status in a L3GD20 Data
  */
uint8_t L3GD20_GetDataStatus(void)
{
  uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read STATUS_REG register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_STATUS_REG_ADDR, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  return tmpreg;
}

/**
* @brief  Calculate the L3GD20 angular data.
* @param  pfData: Data out pointer
* @retval None
*/
void L3GD20_ReadXYZAngRate(float *pfData)
{
  uint8_t tmpbuffer[6] ={0};
  int16_t RawData[3] = {0};
  uint8_t tmpreg = 0;
  float sensitivity = 0;
  int i =0;
  static uint8_t ret;
  
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_CTRL_REG4_ADDR, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  ret = I2Cx_ReadMultipleBytes(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_OUT_ADDR, tmpbuffer, 6);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }

  
  /* check in the control register 4 the data alignment (Big Endian or Little Endian)*/
  if(!(tmpreg & L3GD20_BLE_MSB))
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i+1] << 8) + tmpbuffer[2*i]);
    }
  }
  else
  {
    for(i=0; i<3; i++)
    {
      RawData[i]=(int16_t)(((uint16_t)tmpbuffer[2*i] << 8) + tmpbuffer[2*i+1]);
    }
  }
  
  /* Switch the sensitivity value set in the CRTL4 */
  switch(tmpreg & L3GD20_FULLSCALE_SELECTION)
  {
  case L3GD20_FULLSCALE_250:
    sensitivity=L3GD20_SENSITIVITY_250DPS * 0.001;
    break;
    
  case L3GD20_FULLSCALE_500:
    sensitivity=L3GD20_SENSITIVITY_500DPS * 0.001;
    break;
    
  case L3GD20_FULLSCALE_2000:
    sensitivity=L3GD20_SENSITIVITY_2000DPS * 0.001;
    break;
  }
  /* Divide by sensitivity */
  for(i=0; i<3; i++)
  {
    pfData[i]=(float)(RawData[i] * sensitivity);
  }
}

void L3GD20_ReadTemp(float *pfData)
{
  uint8_t tmp;
  static uint8_t ret;
  static uint8_t rawTempGyro;
  
  /* Read WHO I AM register */
  ret = I2Cx_ReadSingleByte(&hi2c1, L3GD20_I2C_ADDRESS, L3GD20_OUT_TEMP_ADDR, &tmp);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  rawTempGyro = tmp;
  
  pfData[0] = (float)((25 - ((int8_t)(rawTempGyro))) + 25 - 9);
}

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/     
