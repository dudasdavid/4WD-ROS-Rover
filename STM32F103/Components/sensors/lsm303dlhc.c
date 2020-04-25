/**
  ******************************************************************************
  * @file    lsm303dlhc.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    20-November-2014
  * @brief   This file provides a set of functions needed to manage the lsm303dlhc
  *          MEMS accelerometer.
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
#include "lsm303dlhc.h"
#include "i2c_handler.h"

static const ACCELERO_DrvTypeDef Lsm303dlhcDrv =
{
  LSM303DLHC_AccInit,
  LSM303DLHC_AccReadID,
  LSM303DLHC_AccRebootCmd,
  LSM303DLHC_AccZClickITConfig,
  0,
  0,
  0,
  0,
  LSM303DLHC_AccFilterConfig,
  LSM303DLHC_AccFilterCmd,
  LSM303DLHC_AccReadXYZ
};

uint8_t tmpreg2A[8] = {0x00};
uint8_t tmpreg3A = 0x00;
uint8_t tmpregcfgA = 0x00;

extern I2C_HandleTypeDef hi2c1;

void ACCELERO_Init(void)
{
  uint16_t ctrl = 0x0000;
  ACCELERO_InitTypeDef         LSM303DLHC_InitStructure;
  ACCELERO_FilterConfigTypeDef LSM303DLHC_FilterStructure = {0,0,0,0};
  
  if(Lsm303dlhcDrv.ReadID() == I_AM_LMS303DLHC)
  {
    /* Initialize the Accelerometer driver structure */

    /* MEMS configuration ----------------------------------------------------*/
    /* Fill the Accelerometer structure */
    LSM303DLHC_InitStructure.Power_Mode = LSM303DLHC_NORMAL_MODE;
    LSM303DLHC_InitStructure.AccOutput_DataRate = LSM303DLHC_ODR_50_HZ;
    LSM303DLHC_InitStructure.Axes_Enable = LSM303DLHC_AXES_ENABLE;
    LSM303DLHC_InitStructure.AccFull_Scale = LSM303DLHC_FULLSCALE_2G;
    LSM303DLHC_InitStructure.BlockData_Update = LSM303DLHC_BlockUpdate_Continous;
    LSM303DLHC_InitStructure.Endianness = LSM303DLHC_BLE_LSB;
    LSM303DLHC_InitStructure.High_Resolution = LSM303DLHC_HR_ENABLE;
    
    /* Configure MEMS: data rate, power mode, full scale and axes */
    ctrl |= (LSM303DLHC_InitStructure.Power_Mode | LSM303DLHC_InitStructure.AccOutput_DataRate | \
                       LSM303DLHC_InitStructure.Axes_Enable);
    
    ctrl |= ((LSM303DLHC_InitStructure.BlockData_Update | LSM303DLHC_InitStructure.Endianness | \
                      LSM303DLHC_InitStructure.AccFull_Scale | LSM303DLHC_InitStructure.High_Resolution) << 8);
    
    /* Configure the Accelerometer main parameters */
    Lsm303dlhcDrv.Init(ctrl);
    
    /* Fill the Accelerometer LPF structure */
    LSM303DLHC_FilterStructure.HighPassFilter_Mode_Selection =LSM303DLHC_HPM_NORMAL_MODE;
    LSM303DLHC_FilterStructure.HighPassFilter_CutOff_Frequency = LSM303DLHC_HPFCF_16;
    LSM303DLHC_FilterStructure.HighPassFilter_AOI1 = LSM303DLHC_HPF_AOI1_DISABLE;
    LSM303DLHC_FilterStructure.HighPassFilter_AOI2 = LSM303DLHC_HPF_AOI2_DISABLE;
    
    /* Configure MEMS: mode, cutoff frquency, Filter status, Click, AOI1 and AOI2 */
    ctrl = (uint8_t) (LSM303DLHC_FilterStructure.HighPassFilter_Mode_Selection |\
                      LSM303DLHC_FilterStructure.HighPassFilter_CutOff_Frequency|\
                      LSM303DLHC_FilterStructure.HighPassFilter_AOI1|\
                      LSM303DLHC_FilterStructure.HighPassFilter_AOI2);

    /* Configure the Accelerometer LPF main parameters */
    Lsm303dlhcDrv.FilterConfig(ctrl);

  }
}


void ACCELERO_Reset(void)
{
  if(Lsm303dlhcDrv.Reset != NULL)
  {
    Lsm303dlhcDrv.Reset();
  }  
}

/**
  * @brief  Configure Accelerometer click IT. 
  */
void ACCELERO_Click_ITConfig(void)
{
  if(Lsm303dlhcDrv.ConfigIT!= NULL)
  {
    Lsm303dlhcDrv.ConfigIT();
  }
}

/**
  * @brief  Get XYZ axes acceleration.
  * @param  pDataXYZ: Pointer to 3 angular acceleration axes.  
  *                   pDataXYZ[0] = X axis, pDataXYZ[1] = Y axis, pDataXYZ[2] = Z axis
  */
void ACCELERO_GetXYZ(float *pfDataXYZ)
{
  //float SwitchXY = 0;
  
  if(Lsm303dlhcDrv.GetXYZ!= NULL)
  {
    Lsm303dlhcDrv.GetXYZ(pfDataXYZ);
    
    /* Switch X and Y Axes in case of LSM303DLHC MEMS */
    //if(AccelerometerDrv == &Lsm303dlhcDrv)
    //{ 
    //  SwitchXY  = pfDataXYZ[0];
    //  pfDataXYZ[0] = pfDataXYZ[1];
    //  
    //  /* Invert Y Axis to be conpliant with LIS3DSH */
    //  pfDataXYZ[1] = -SwitchXY;
    //} 
  }
}








/**
  * @}
  */

/** @defgroup LSM303DLHC_Private_Functions
  * @{
  */

/**
  * @brief  Set LSM303DLHC Initialization.
  * @param  InitStruct: Init parameters
  * @retval None
  */
void LSM303DLHC_AccInit(uint16_t InitStruct)
{  
  static uint8_t ctrl1 = 0x00;
  static uint8_t ctrl4 = 0x00;
  static uint8_t ret;

  /* Write value to ACC MEMS CTRL_REG1 register */
  ctrl1 = InitStruct & 0x00FF;
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG1_A, &ctrl1, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Write value to ACC MEMS CTRL_REG4 register */
  ctrl4 = (InitStruct & 0xFF00) >> 8;
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrl4, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Read LSM303DLHC ID.
  * @param  None
  * @retval ID 
  */
uint8_t LSM303DLHC_AccReadID(void)
{  
  uint8_t buf;
  static uint8_t ret;
  
  /* Read value at Who am I register address */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_WHO_AM_I_ADDR, &buf);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  return buf;
}

/**
  * @brief  Reboot memory content of LSM303DLHC
  * @param  None
  * @retval None
  */
void LSM303DLHC_AccRebootCmd(void)
{
  uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read CTRL_REG5 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Enable or Disable the reboot memory */
  tmpreg |= LSM303DLHC_BOOT_REBOOTMEMORY;
  
  /* Write value to ACC MEMS CTRL_REG5 register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Set High Pass Filter Modality
  * @param  FilterStruct: contains data for filter config
  * @retval None
  */
void LSM303DLHC_AccFilterConfig(uint8_t FilterStruct) 
{
  static uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read CTRL_REG2 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  tmpreg &= 0x0C;
  tmpreg |= FilterStruct;
  
  /* Write value to ACC MEMS CTRL_REG2 register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Enable or Disable High Pass Filter
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: LSM303DLHC_HIGHPASSFILTER_DISABLE 
  *         @arg: LSM303DLHC_HIGHPASSFILTER_ENABLE
  * @retval None
  */
void LSM303DLHC_AccFilterCmd(uint8_t HighPassFilterState)
{
  uint8_t tmpreg;
  static uint8_t ret;
  
  /* Read CTRL_REG2 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  tmpreg &= 0xF7;
  tmpreg |= HighPassFilterState;
  
  /* Write value to ACC MEMS CTRL_REG2 register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Read X, Y & Z Acceleration values 
  * @param  pData: Data out pointer
  * @retval None
  */
void LSM303DLHC_AccReadXYZ(float* pfData)
{
  int16_t pnRawData[3];
  static uint8_t ctrlx[2]={0,0};
  uint8_t buffer[6];
  uint8_t i = 0;
  static uint8_t sensitivity = 0;
  static uint8_t ret;
  
  /* Read the acceleration control register content */ 
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG4_A, &ctrlx[0]);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG5_A, &ctrlx[1]);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Read output register X, Y & Z acceleration */
  ret = I2Cx_ReadMultipleBytes(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_A, buffer, 6);
  //ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_L_A, &buffer[0]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_X_H_A, &buffer[1]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_A, &buffer[2]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_A, &buffer[3]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_A, &buffer[4]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_A, &buffer[5]);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  

  /* Check in the control register4 the data alignment*/
  if(!(ctrlx[0] & LSM303DLHC_BLE_MSB)) 
  {
    for(i=0; i<3; i++)
    {
      //pnRawData[i]=(int16_t)(buffer[2*i] | (buffer[2*i+1] << 8)) >> 4;
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i+1] << 8) + buffer[2*i]);
    }
  }
  else /* Big Endian Mode */
  {
    for(i=0; i<3; i++)
    {
      pnRawData[i]=((int16_t)((uint16_t)buffer[2*i] << 8) + buffer[2*i+1]);
    }
  }
  
  /* Normal mode */
  /* Switch the sensitivity value set in the CRTL4 */
  switch(ctrlx[0] & LSM303DLHC_FULLSCALE_16G)
  {
  case 0x00:
    sensitivity = LSM303DLHC_ACC_SENSITIVITY_2G;
    break;
  case 0x10:
    sensitivity = LSM303DLHC_ACC_SENSITIVITY_4G;
    break;
  case 0x20:
    sensitivity = LSM303DLHC_ACC_SENSITIVITY_8G;
    break;
  case 0x30:
    sensitivity = LSM303DLHC_ACC_SENSITIVITY_16G;
    break;
  }
  
  /* Obtain the mg value for the three axis */
  for(i=0; i<3; i++)
  {
    pfData[i]=(pnRawData[i] * sensitivity / 16)*0.00980665;
  }
}

/**
  * @brief  Enable or Disable High Pass Filter on CLick
  * @param  HighPassFilterState: new state of the High Pass Filter feature.
  *      This parameter can be: 
  *         @arg: LSM303DLHC_HPF_CLICK_DISABLE 
  *         @arg: LSM303DLHC_HPF_CLICK_ENABLE
  * @retval None
  */
void LSM303DLHC_AccFilterClickCmd(uint8_t HighPassFilterClickState)
{
  uint8_t tmpreg = 0x00;
  static uint8_t ret;
  
  /* Read CTRL_REG2 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  tmpreg &= ~(LSM303DLHC_HPF_CLICK_ENABLE);
  tmpreg |= HighPassFilterClickState;
  
  /* Write value to ACC MEMS CTRL_REG2 regsister */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG2_A, &tmpreg, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief Enable LSM303DLHC Interrupt1
  * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303DLHC_IT1_CLICK
  *         @arg   LSM303DLHC_IT1_AOI1
  *         @arg   LSM303DLHC_IT1_AOI2
  *         @arg   LSM303DLHC_IT1_DRY1
  *         @arg   LSM303DLHC_IT1_DRY2
  *         @arg   LSM303DLHC_IT1_WTM
  *         @arg   LSM303DLHC_IT1_OVERRUN
  * @retval None
  */
void LSM303DLHC_AccIT1Enable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read CTRL_REG3 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Enable IT1 */
  tmpval |= LSM303DLHC_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief Disable LSM303DLHC Interrupt1
  * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be disabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303DLHC_IT1_CLICK
  *         @arg   LSM303DLHC_IT1_AOI1
  *         @arg   LSM303DLHC_IT1_AOI2
  *         @arg   LSM303DLHC_IT1_DRY1
  *         @arg   LSM303DLHC_IT1_DRY2
  *         @arg   LSM303DLHC_IT1_WTM
  *         @arg   LSM303DLHC_IT1_OVERRUN
  * @retval None
  */
void LSM303DLHC_AccIT1Disable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read CTRL_REG3 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Disable IT1 */
  tmpval &= ~LSM303DLHC_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG3_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief Enable LSM303DLHC Interrupt2 
  * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be enabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303DLHC_IT2_CLICK
  *         @arg   LSM303DLHC_IT2_INT1
  *         @arg   LSM303DLHC_IT2_INT2
  *         @arg   LSM303DLHC_IT2_BOOT
  *         @arg   LSM303DLHC_IT2_ACT
  *         @arg   LSM303DLHC_IT2_HLACTIVE
  * @retval None
  */
void LSM303DLHC_AccIT2Enable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read CTRL_REG3 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Enable IT2 */
  tmpval |= LSM303DLHC_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief Disable LSM303DLHC Interrupt2
  * @param  LSM303DLHC_IT: specifies the LSM303DLHC interrupt source to be disabled.
  *           This parameter can be any combination of the following values: 
  *         @arg   LSM303DLHC_IT2_CLICK
  *         @arg   LSM303DLHC_IT2_INT1
  *         @arg   LSM303DLHC_IT2_INT2
  *         @arg   LSM303DLHC_IT2_BOOT
  *         @arg   LSM303DLHC_IT2_ACT
  *         @arg   LSM303DLHC_IT2_HLACTIVE
  * @retval None
  */
void LSM303DLHC_AccIT2Disable(uint8_t LSM303DLHC_IT)
{
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read CTRL_REG3 register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  
  /* Disable IT2 */
  tmpval &= ~LSM303DLHC_IT;
  
  /* Write value to MEMS CTRL_REG3 register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CTRL_REG6_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  INT1 interrupt enable
  * @param  ITCombination: Or or And combination
  *         ITAxes: Axes to be enabled 
  * @retval None
  */
void LSM303DLHC_AccINT1InterruptEnable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read INT1_CFR register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Enable the selected interrupt */
  tmpval |= (ITAxes | ITCombination);
  
  /* Write value to MEMS INT1_CFR register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  INT1 interrupt disable
  * @param  ITCombination: Or or And combination
  *         ITAxes: Axes to be enabled 
  * @retval None
  */
void LSM303DLHC_AccINT1InterruptDisable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read INT1_CFR register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Disable the selected interrupt */
  tmpval &= ~(ITAxes | ITCombination);
  
  /* Write value to MEMS INT1_CFR register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_INT1_CFG_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  INT2 interrupt enable
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled 
  * @retval None
  */
void LSM303DLHC_AccINT2InterruptEnable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read INT2_CFR register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Enable the selected interrupt */
  tmpval |= (ITAxes | ITCombination);
  
  /* Write value to MEMS INT2_CFR register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  INT2 interrupt config
  * @param  ITCombination: Or or And combination
  *         ITAxes: axes to be enabled 
  * @retval None
  */
void LSM303DLHC_AccINT2InterruptDisable(uint8_t ITCombination, uint8_t ITAxes)
{  
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read INT2_CFR register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Disable the selected interrupt */
  tmpval &= ~(ITAxes | ITCombination);
  
  /* Write value to MEMS INT2_CFR register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_INT2_CFG_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
}

/**
  * @brief  Click interrupt enable
  * @param  ITClick: the selected interrupt to enable
  * @retval None
  */
void LSM303DLHC_AccClickITEnable(uint8_t ITClick)
{  
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read CLICK_CFR register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Enable the selected interrupt */
  tmpval |= ITClick;
  
  /* Write value to MEMS CLICK CFG register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Configure Click Threshold on Z axis */
  tmpval = 0x0A;
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CLICK_THS_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Configure Time Limit */
  tmpval = 0x05;
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_TIME_LIMIT_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Configure Latency */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_TIME_LATENCY_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Configure Click Window */
  tmpval = 0x32;
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_TIME_WINDOW_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Click interrupt disable
  * @param  ITClick: the selected click interrupt to disable
  * @retval None
  */
void LSM303DLHC_AccClickITDisable(uint8_t ITClick)
{  
  uint8_t tmpval = 0x00;
  static uint8_t ret;
  
  /* Read CLICK_CFR register */
  ret = I2Cx_ReadSingleByte(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, &tmpval);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Disable the selected interrupt */
  tmpval &= ~ITClick;
  
  /* Write value to MEMS CLICK_CFR register */
  ret = I2Cx_WriteData(&hi2c1, ACC_I2C_ADDRESS, LSM303DLHC_CLICK_CFG_A, &tmpval, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  Click on Z axis interrupt config
  * @param  None
  * @retval None
  */
void LSM303DLHC_AccZClickITConfig(void)
{  
 
  /* Select click IT as INT1 interrupt */
  LSM303DLHC_AccIT1Enable(LSM303DLHC_IT1_CLICK);
  
  /* Enable High pass filter for click IT */
  LSM303DLHC_AccFilterClickCmd(LSM303DLHC_HPF_CLICK_ENABLE);
  
  /* Enable simple click IT on Z axis, */
  LSM303DLHC_AccClickITEnable(LSM303DLHC_Z_SINGLE_CLICK);
}



void MAGNET_Init(void)
{
  LACCELERO_InitTypeDef LSM303DLHC_MAG_InitStruct;
  
  /* Configure MEMS magnetometer main parameters: temp, working mode, full Scale and Data rate */
  LSM303DLHC_MAG_InitStruct.Temperature_Sensor = LSM303DLHC_TEMPSENSOR_ENABLE;
  LSM303DLHC_MAG_InitStruct.MagOutput_DataRate =LSM303DLHC_ODR_30_HZ ;
  LSM303DLHC_MAG_InitStruct.MagFull_Scale = LSM303DLHC_FS_4_0_GA;
  LSM303DLHC_MAG_InitStruct.Working_Mode = LSM303DLHC_CONTINUOS_CONVERSION;
  LSM303DLHC_MagInit(&LSM303DLHC_MAG_InitStruct);
}

void LSM303DLHC_MagInit(LACCELERO_InitTypeDef *LSM303DLHC_InitStruct)
{  
  uint8_t cra_regm = 0x00, crb_regm = 0x00, mr_regm = 0x00;
  uint8_t ret;
  
  /* Configure the low level interface ---------------------------------------*/
  //LSM303DLHC_LowLevel_Init();
  
  /* Configure MEMS: temp and Data rate */
  cra_regm |= (uint8_t) (LSM303DLHC_InitStruct->Temperature_Sensor | LSM303DLHC_InitStruct->MagOutput_DataRate);
    
  /* Configure MEMS: full Scale */
  crb_regm |= (uint8_t) (LSM303DLHC_InitStruct->MagFull_Scale);
      
  /* Configure MEMS: working mode */
  mr_regm |= (uint8_t) (LSM303DLHC_InitStruct->Working_Mode);
                    
  /* Write value to Mag MEMS CRA_REG regsister */
  ret = I2Cx_WriteData(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_CRA_REG_M, &cra_regm, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Write value to Mag MEMS CRB_REG regsister */
  ret = I2Cx_WriteData(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &crb_regm, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }

  /* Write value to Mag MEMS MR_REG regsister */
  ret = I2Cx_WriteData(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_MR_REG_M, &mr_regm, 1);
  
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
}

/**
  * @brief  calculate the magnetic field Magn.
* @param  pfData: pointer to the data out
  * @retval None
  */
void LSM303DLHC_MagReadXYZ (float* pfData)
{
  uint8_t buffer[6] = {0};
  uint8_t CTRLB = 0;
  float Magn_Sensitivity_XY = 0, Magn_Sensitivity_Z = 0;
  uint8_t i =0;
  uint8_t ret;

    ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_CRB_REG_M, &CTRLB);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Read output register X, Y & Z magnetic value */
  ret = I2Cx_ReadMultipleBytes(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_OUT_M, buffer, 6);
  //ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_L_M, &buffer[0]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_OUT_X_H_M, &buffer[1]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_L_M, &buffer[2]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_OUT_Y_H_M, &buffer[3]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_L_M, &buffer[4]);
  //ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_OUT_Z_H_M, &buffer[5]);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  
  /* Switch the sensitivity set in the CRTLB*/
  switch(CTRLB & 0xE0)
  {
  case LSM303DLHC_FS_1_3_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_3Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_3Ga;
    break;
  case LSM303DLHC_FS_1_9_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_1_9Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_1_9Ga;
    break;
  case LSM303DLHC_FS_2_5_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_2_5Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_2_5Ga;
    break;
  case LSM303DLHC_FS_4_0_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4Ga;
    break;
  case LSM303DLHC_FS_4_7_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_4_7Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_4_7Ga;
    break;
  case LSM303DLHC_FS_5_6_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_5_6Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_5_6Ga;
    break;
  case LSM303DLHC_FS_8_1_GA:
    Magn_Sensitivity_XY = LSM303DLHC_M_SENSITIVITY_XY_8_1Ga;
    Magn_Sensitivity_Z = LSM303DLHC_M_SENSITIVITY_Z_8_1Ga;
    break;
  }
  
  // X, Z, Y is the output order, thanks ST
  pfData[0]=((int16_t)(((uint16_t)buffer[0] << 8) + buffer[1]))/Magn_Sensitivity_XY;
  pfData[2]=((int16_t)(((uint16_t)buffer[2] << 8) + buffer[3]))/Magn_Sensitivity_Z;
  pfData[1]=((int16_t)(((uint16_t)buffer[4] << 8) + buffer[5]))/Magn_Sensitivity_XY;
  
}

void LSM303DLHC_MagReadTemp (float* pfData)
{
  //static int16_t tempC;
  static int16_t rawTemp;
  static uint8_t buffer[2] = {0};
  static uint8_t ret;
  
  ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_TEMP_OUT_L_M, &buffer[0]);
  ret = I2Cx_ReadSingleByte(&hi2c1, MAG_I2C_ADDRESS, LSM303DLHC_TEMP_OUT_H_M, &buffer[1]);
  if ( ret != HAL_OK ) {
    __asm("NOP");
  }
  

  rawTemp=(((uint16_t)buffer[1] << 8) + buffer[0]);
  
  //rawTemp = (((buffer[1] << 8) + buffer[0]) >> 4); //temperature raw
  //tempC = rawTemp / 8 / 8 + 25; //temperature in celsuis + offset
  
  pfData[0] = rawTemp / 8.0 / 8.0;
  //pData[0] = (int16_t)((buffer[1] << 4) + (buffer[0] >> 4));
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
