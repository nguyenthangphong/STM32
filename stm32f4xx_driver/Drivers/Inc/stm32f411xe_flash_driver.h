#ifndef INC_STM32F411XE_FLASH_DRIVER_H_
#define INC_STM32F411XE_FLASH_DRIVER_H_

#include "stm32f411xe.h"
#include "stm32f411xe_gpio_driver.h"
#include "stm32f411xe_def.h"

/* Latency */
#define FLASH_LATENCY_0                             (0x00000000U)                       /*!< LATENCY = 0 */
#define FLASH_LATENCY_1                             (0x00000001U)                       /*!< LATENCY = 1 */
#define FLASH_LATENCY_2                             (0x00000002U)                       /*!< LATENCY = 2 */
#define FLASH_LATENCY_3                             (0x00000003U)                       /*!< LATENCY = 3 */
#define FLASH_LATENCY_4                             (0x00000004U)                       /*!< LATENCY = 4 */
#define FLASH_LATENCY_5                             (0x00000005U)                       /*!< LATENCY = 5 */
#define FLASH_LATENCY_6                             (0x00000006U)                       /*!< LATENCY = 6 */
#define FLASH_LATENCY_7                             (0x00000007U)                       /*!< LATENCY = 7 */

#endif /* INC_STM32F411XE_FLASH_DRIVER_H_ */