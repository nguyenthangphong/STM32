#ifndef INC_STM32F411XE_DEF_H_
#define INC_STM32F411XE_DEF_H_

#define ENABLE                  (0x00000001U)
#define DISABLE                 (0x00000000U)
#define SET                     ENABLE
#define RESET                   DISABLE
#define READY                   ENABLE
#define NOT_READY               DISABLE
#define GPIO_PIN_SET            ENABLE
#define GPIO_PIN_RESET          DISABLE
#define FLAG_SET                SET
#define FLAG_RESET              RESET
#define UNUSED(x)               ((void)x)

typedef enum
{
    STATUS_OK        = 0u,
    STATUS_ERROR     = 1u
} e_StatusTypeDef_t;

#endif /* INC_STM32F411XE_DEF_H_ */