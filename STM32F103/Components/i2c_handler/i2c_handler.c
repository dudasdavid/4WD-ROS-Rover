/* Includes ------------------------------------------------------------------*/
#include "i2c_handler.h"

static uint16_t i2cerror;
static uint16_t i2cok;

HAL_StatusTypeDef I2Cx_ReadSingleByte(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg, uint8_t *value)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(hi2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, value, 1, I2Cx_TIMEOUT_MAX);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    i2cerror++;
  }
  else {
    i2cok++;
  }
  return status;
}


HAL_StatusTypeDef I2Cx_ReadMultipleBytes(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg, uint8_t *value, uint8_t len)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Read(hi2c, Addr, Reg, I2C_MEMADD_SIZE_8BIT, value, len, I2Cx_TIMEOUT_MAX);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    i2cerror++;
  }
  else {
    i2cok++;
  }
  return status;
}


HAL_StatusTypeDef I2Cx_WriteData(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg, uint8_t *Value, uint8_t len)
{
  HAL_StatusTypeDef status = HAL_OK;
  
  status = HAL_I2C_Mem_Write(hi2c, Addr, (uint16_t)Reg, I2C_MEMADD_SIZE_8BIT, Value, len, I2Cx_TIMEOUT_MAX);
  
  /* Check the communication status */
  if(status != HAL_OK)
  {
    i2cerror++;
  }
  else {
    i2cok++;
  }
  return status;
}

uint16_t GetCommOk(void){
  return i2cok;
}

uint16_t GetCommError(void){
  return i2cerror;
}