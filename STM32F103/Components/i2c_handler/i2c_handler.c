/* Includes ------------------------------------------------------------------*/
#include "i2c_handler.h"

static uint16_t i2cerror;
static uint16_t i2cok;

uint8_t I2Cx_ReadData(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg)
{
  HAL_StatusTypeDef status = HAL_OK;
  uint8_t value = 0;
  
  status = HAL_I2C_Mem_Read(hi2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, &value, 1, I2Cx_TIMEOUT_MAX);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    i2cerror++;
    __asm("NOP");
    //I2Cx_Error();
  }
  else {
    i2cok++;
  }
  return value;
}


uint8_t I2Cx_ReadDataMultiple(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg, uint8_t *value, uint8_t len)
{
  HAL_StatusTypeDef status = HAL_OK;
  //uint8_t value = 0;
  
  status = HAL_I2C_Mem_Read(hi2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, value, len, I2Cx_TIMEOUT_MAX);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    i2cerror++;
    __asm("NOP");
    //I2Cx_Error();
  }
  else {
    i2cok++;
  }
  return 0;
}


uint8_t I2Cx_WriteData(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg, uint8_t *Value, uint8_t len)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(hi2c, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Value, len, I2Cx_TIMEOUT_MAX);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    /* Execute user timeout callback */
    i2cerror++;
    //I2Cx_Error();
  }
  return 0;
}