#include <F28335_pwm.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include <F28335Serial.h>

//#define DAN_PWM_CARRIER_HZ	80000L
//#define DAN_PWM_CARRIER_HZ	40000L  // The limit of the A3953 chip is 70Khz
#define DAN_PWM_CARRIER_HZ	20000L  // The limit of the A3953 chip is 70Khz

#define DAN_PWM_TPS			0x0
#define DAN_PWM_TCLK		(HISPCLK_HZ >> (DAN_PWM_TPS)) //HISPCLK is at 150 MHz
#define DAN_PWM_PR			(DAN_PWM_TCLK / DAN_PWM_CARRIER_HZ)
#define DAN_PWM_DUTY50		(DAN_PWM_PR >> 1)

int PWM3setupasRCservo = 0;
int PWM4setupasRCservo = 0;
int PWM5setupasRCservo = 0;
int dummy2 = 0;


void init_PWMandDIR(int ep)
{
	switch (ep) {


	case 1:

		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;
		GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
		EDIS;
		dummy2 = DAN_PWM_PR;
		EPwm1Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm1Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm1Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm1Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm1Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm1Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm1Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm1Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm1Regs.CMPA.half.CMPA = 0;  // Start off zero percent duty cycle
		EPwm1Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm1Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
		EPwm1Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case 2:

		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;
		GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
		EDIS;

		EPwm2Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm2Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm2Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm2Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm2Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm2Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm2Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm2Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm2Regs.CMPA.half.CMPA = 0;  // Start off with a square wave
		EPwm2Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm2Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
		EPwm2Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		break;

	case 3:

		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;
		GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
		EDIS;

		EPwm3Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm3Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm3Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm3Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm3Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm3Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm3Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm3Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm3Regs.CMPA.half.CMPA = 0;  // Start off with a square wave
		EPwm3Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm3Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
		EPwm3Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		PWM3setupasRCservo = 0;
		break;

	case 4:

		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;
		GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO7 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
		EDIS;

		EPwm4Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm4Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm4Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm4Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm4Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm4Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm4Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm4Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm4Regs.CMPA.half.CMPA = 0;  // Start off with a square wave
		EPwm4Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm4Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
		EPwm4Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		PWM4setupasRCservo = 0;
		break;

	case 5:

		EALLOW;
		GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;
		GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;
		GpioCtrlRegs.GPADIR.bit.GPIO19 = 1;
		GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
		EDIS;

		EPwm5Regs.TBPRD = DAN_PWM_PR;  // Set to Period for 20Khz carrier
		EPwm5Regs.TBPHS.half.TBPHS = 0;  // not using Phase
		EPwm5Regs.TBCTL.bit.FREE_SOFT = 2; // don't stop on emulation suspend
		EPwm5Regs.TBCTL.bit.CLKDIV = 0; // Base clk set to SYSCLK
		EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0; // Base clk set to SYSCLK
		EPwm5Regs.TBCTL.bit.SYNCOSEL = 3; //  SYNCO disabled
		EPwm5Regs.TBCTL.bit.PRDLD = 0; // Use Shadow
		EPwm5Regs.TBCTL.bit.PHSEN = 0; // Phase disabled
		EPwm5Regs.TBCTL.bit.CTRMODE = 0; // Count up

		EPwm5Regs.CMPA.half.CMPA = 0;  // Start off with a square wave
		EPwm5Regs.CMPCTL.bit.LOADAMODE = 1; // Load on CTR=PRD
		EPwm5Regs.AQCTLA.bit.CAU = 1; // Clear pin when CTR=CMPA
		EPwm5Regs.AQCTLA.bit.PRD = 2; // Set pin when CTR=PRD

		PWM5setupasRCservo = 0;
		break;

	}

}

/* PWM_out(): update PWM's duty cycle -10.0<u<10.0
 *
 * parameters:
 *     pwm = PWM to update
 *     u   = duty cycle (-10 < u < 10)
 */
void PWMandDIR_out(int ep, float u)
{
	unsigned int val;

	if (u > 10.0F) {
		u = 10.0F;
	} else if (u < -10.0F) {
		u = -10.0F;
	}

	val = DAN_PWM_PR*fabsf(u)/10.0F;

	switch (ep) {
	case 1:
		EPwm1Regs.CMPA.half.CMPA = val;
		if (u >= 0) {
			GpioDataRegs.GPASET.bit.GPIO1 = 1;
		} else {
			GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;
		}
		break;
	case 2:
		EPwm2Regs.CMPA.half.CMPA = val;
		if (u >= 0) {
			GpioDataRegs.GPASET.bit.GPIO3 = 1;
		} else {
			GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
		}
		break;
	case 3:
		if (PWM3setupasRCservo == 0) {
			EPwm3Regs.CMPA.half.CMPA = val;
			if (u >= 0) {
				GpioDataRegs.GPASET.bit.GPIO5 = 1;
			} else {
				GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;
			}
		}
		break;
	case 4:
		if (PWM4setupasRCservo == 0) {
			EPwm4Regs.CMPA.half.CMPA = val;
			if (u >= 0) {
				GpioDataRegs.GPASET.bit.GPIO7 = 1;
			} else {
				GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
			}
		}
		break;
	case 5:
		if (PWM5setupasRCservo == 0) {
			EPwm5Regs.CMPA.half.CMPA = val;
			if (u >= 0) {
				GpioDataRegs.GPASET.bit.GPIO19 = 1;
			} else {
				GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;
			}
		}
		break;
	}
}
