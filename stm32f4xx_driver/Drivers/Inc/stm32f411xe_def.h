#ifndef INC_STM32F411XE_DEF_H_
#define INC_STM32F411XE_DEF_H_

/* Exported_macro */
#define SET_REGISTER(register_name, clear_mask, set_mask) \
    (((register_name) & (~(clear_mask))) | ((set_mask) & (clear_mask)))

typedef enum
{
    STATUS_OK        = 0u,
    STATUS_ERROR     = 1u
} e_StatusTypeDef_t;

#endif /* INC_STM32F411XE_DEF_H_ */