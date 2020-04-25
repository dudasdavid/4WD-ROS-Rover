#include <stdint.h>
#include "stm32f1xx_hal.h"

#define I2Cx_TIMEOUT_MAX                        100

#define DISCOVERY_I2Cx_FORCE_RESET()            __HAL_RCC_I2C1_FORCE_RESET()
#define DISCOVERY_I2Cx_RELEASE_RESET()          __HAL_RCC_I2C1_RELEASE_RESET()

/* I2C interrupt requests */
#define DISCOVERY_I2Cx_EV_IRQn                  I2C1_EV_IRQn
#define DISCOVERY_I2Cx_ER_IRQn                  I2C1_ER_IRQn

#define DISCOVERY_I2Cx                          I2C1
#define DISCOVERY_I2Cx_CLOCK_ENABLE()           __HAL_RCC_I2C1_CLK_ENABLE()
#define DISCOVERY_I2Cx_GPIO_PORT                GPIOB                       /* GPIOB */
#define DISCOVERY_I2Cx_SCL_PIN                  GPIO_PIN_8                  /* PB.08 */
#define DISCOVERY_I2Cx_SDA_PIN                  GPIO_PIN_9                  /* PB.09 */
#define DISCOVERY_I2Cx_GPIO_CLK_ENABLE()        __HAL_RCC_GPIOB_CLK_ENABLE() 
#define DISCOVERY_I2Cx_GPIO_CLK_DISABLE()       __HAL_RCC_GPIOB_CLK_DISABLE() 
#define DISCOVERY_I2Cx_AF                       GPIO_AF4_I2C1



HAL_StatusTypeDef I2Cx_ReadSingleByte(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg, uint8_t *value);
HAL_StatusTypeDef I2Cx_ReadMultipleBytes(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg, uint8_t *value, uint8_t len);
HAL_StatusTypeDef I2Cx_WriteData(I2C_HandleTypeDef *hi2c, uint16_t Addr, uint8_t Reg, uint8_t *Value, uint8_t len);
