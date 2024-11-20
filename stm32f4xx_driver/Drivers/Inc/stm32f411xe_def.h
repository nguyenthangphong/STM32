#ifndef INC_STM32F411XE_DEF_H_
#define INC_STM32F411XE_DEF_H_

#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            ENABLE
#define GPIO_PIN_RESET          DISABLE
#define FLAG_SET                SET
#define FLAG_RESET              RESET
#define UNUSED(x)               ((void)x)

#define SET_REGISTER(register_name, clear_mask, set_mask)   (((register_name) & (~(clear_mask))) | ((set_mask) & (clear_mask)))
#define GET_REGISTER(register_name, clear_mask)             ((register_name) & (clear_mask))
#define SET_BIT(register_name, bit_name)                    ((register_name) |= (bit_name))

typedef enum
{
    STATUS_OK        = 0u,
    STATUS_ERROR     = 1u
} e_StatusTypeDef_t;

#endif /* INC_STM32F411XE_DEF_H_ */