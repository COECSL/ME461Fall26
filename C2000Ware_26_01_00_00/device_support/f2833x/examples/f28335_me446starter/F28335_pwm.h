#ifndef __28335_PWM_H__
#define __28335_PWM_H__

#define PWM_CARRIER_HZ	20000L

#define PWM_TPS			0x0
#define PWM_TCLK		(HISPCLK_HZ >> (PWM_TPS))
#define PWM_PR			(PWM_TCLK / PWM_CARRIER_HZ)
#define PWM_DUTY50		(PWM_PR >> 1)


void init_PWMandDIR(int ep);
void PWMandDIR_out(int ep, float u);


#endif /* __2808_PWM_H__ */

