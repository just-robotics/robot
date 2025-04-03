#ifndef __GPIO_H__
#define __GPIO_H__

#ifdef __cplusplus
extern "C" {
#endif


#include "main.h"


void MX_GPIO_Init(void);

void set_pwm_lf(uint8_t pwm); //pa0
void set_pwm_lb(uint8_t pwm); //pa1
void set_pwm_rf(uint8_t pwm); //pb4
void set_pwm_rb(uint8_t pwm); //pb5


#ifdef __cplusplus
}
#endif
#endif /*__ GPIO_H__ */
