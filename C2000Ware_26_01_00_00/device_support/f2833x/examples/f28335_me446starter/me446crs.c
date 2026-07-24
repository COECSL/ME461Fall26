//###########################################################################
//
// FILE:   Example_2833xCpuTimer.c
//
// TITLE:  Cpu Timer Example
//
//! \addtogroup f2833x_example_list
//! <h1>Cpu Timer (cpu_timer)</h1>
//!
//! This example configures CPU Timer0, 1, and 2 and increments
//! a counter each time the timers asserts an interrupt.
//!
//! \b Watch \b Variables \n
//! - CputTimer0.InterruptCount
//! - CpuTimer1.InterruptCount
//! - CpuTimer2.InterruptCount
//
//###########################################################################
// $TI Release: $
// $Release Date: $
// $Copyright:
// Copyright (C) 2009-2022 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without 
// modification, are permitted provided that the following conditions 
// are met:
// 
//   Redistributions of source code must retain the above copyright 
//   notice, this list of conditions and the following disclaimer.
// 
//   Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the 
//   documentation and/or other materials provided with the   
//   distribution.
// 
//   Neither the name of Texas Instruments Incorporated nor the names of
//   its contributors may be used to endorse or promote products derived
//   from this software without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// $
//###########################################################################

//
// Included Files
//
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File
#include "DSP2833x_Device.h"

__interrupt void TXAINT_data_sent(void);
__interrupt void RXAINT_recv_ready(void);

//#include "28335_eqep.h"
#include "F28335Serial.h"
#include "F28335_pwm.h"
#include "F28335_spi.h"
//#include "28335_inits.h"

/* general constants */
#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398
#define GRAV        9.81
//
// Function Prototype statements
//
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SPI_RXint(void);
__interrupt void SWI_isr(void);

void updateData(void);
void releaseBrk(void);
void control(void);
void printing(void);

extern int UARTprint;
//float printtheta1motor = 0;
//float printtheta2motor = 0;
//float printtheta3motor = 0;
extern int SPIenc_state;
extern long SPIenc1_reading;
extern long SPIenc2_reading;
extern long SPIenc3_reading;
extern long SPIenc4_reading;
extern long SPIenc5_reading;
extern float offset_Enc2_rad;
extern float offset_Enc3_rad;

unsigned long Main_timeint = 0;

float Main_Enc1_rad = 0;
float Main_Enc2_rad = 0;
float Main_Enc3_rad = 0;
float Main_Enc4_rad = 0;
float Main_Enc5_rad = 0;
float testvar = 0;
float testvar2 = 0;
//eqep_t Main_enc1;
//eqep_t Main_enc2;
float Main_value_enc1 = 0;
float Main_value_enc2 = 0;

float Main_v1 = 0;
float Main_v1old1 = 0;
float Main_v1old2 = 0;
float Main_angle1_ref = 0.0;
float Main_u1 = 0;
float Main_Enc1_rad_old = 0;
float Main_e1 = 0, Main_e1_thresh = 0.1;

float Main_v2 = 0;
float Main_v2old1 = 0;
float Main_v2old2 = 0;
float Main_angle2_ref = 0.0;
float Main_u2 = 0;
float Main_Enc2_rad_old = 0;
float Main_e2 = 0, Main_e2_thresh = 0.1;

float Main_v3 = 0;
float Main_v3old1 = 0;
float Main_v3old2 = 0;
float Main_angle3_ref = 0.0;
float Main_u3 = 0;
float Main_Enc3_rad_old = 0;
float Main_e3 = 0, Main_e3_thresh = 0.1;

float Main_v4 = 0;
float Main_v4old1 = 0;
float Main_v4old2 = 0;
float Main_angle4_ref = 0.0;
float Main_u4 = 0;
float Main_Enc4_rad_old = 0;
float Main_e4 = 0, Main_e4_thresh = 0.1;

float Main_v5 = 0;
float Main_v5old1 = 0;
float Main_v5old2 = 0;
float Main_angle5_ref = 0.0;
float Main_u5 = 0;
float Main_Enc5_rad_old = 0;
float Main_e5 = 0, Main_e5_thresh = 0.1;

float Main_Kp1 = 30;
float Main_Kp2 = 30;
float Main_Kp3 = 30;
float Main_Kp4 = 55;
float Main_Kp5 = 55;

float Main_Kd1 = 1;
float Main_Kd2 = 1;
float Main_Kd3 = 1;
float Main_Kd4 = 1;
float Main_Kd5 = 1;

float Main_Kp1_long = 15;
float Main_Kp2_long = 15;
float Main_Kp3_long = 15;
float Main_Kp4_long = 55;
float Main_Kp5_long = 55;
float Main_Kp1_short = 25;
float Main_Kp2_short = 25;
float Main_Kp3_short = 25;
float Main_Kp4_short = 55;
float Main_Kp5_short = 55;


#define NUM_SEND_QUEUES 120
#define MAX_SEND_LENGTH 1600
#define MAX_VAR_NUM 10

void lab(float theta1motor,float theta2motor,float theta3motor,float *tau1,float *tau2,float *tau3, int error);
float Main_labtau1 = 0;
float Main_labtau2 = 0;
float Main_labtau3 = 0;
int Main_safetymode = 0;

float Main_thetaDH1 = 0;
float Main_thetaDH2 = 0;
float Main_thetaDH3 = 0;
// forward kinematics
float Main_cosq1 = 0;
float Main_sinq1 = 0;
float Main_cosq2 = 0;
float Main_sinq2 = 0;
float Main_cosq3 = 0;
float Main_sinq3 = 0;
float Main_xk = 0;
float Main_yk = 0;
float Main_zk = 0;

float Main_u1Integral = 0;
float Main_u2Integral = 0;
float Main_u3Integral = 0;

float Main_IntegralSum = 0;

float Main_u1_old = 0;
float Main_u2_old = 0;
float Main_u3_old = 0;


#define DHTHETA1_MAX PI*0.6667
#define DHTHETA1_MIN -PI*0.6667
#define DHTHETA2_MAX 0.15
#define DHTHETA2_MIN -2.3
#define DHTHETA3_MAX 2.3
#define DHTHETA3_MIN 0.15
#define Z_MIN 4.0


int shutoffEnc5error  = 0;
int numSWIcalls = 0;

char Main_SendArray[128];
char Main_SendArray2[128];
float Main_tempf=0;
int Main_j = 0;
extern int Main_i;
extern int MatlabCommand;
extern int Main_memcount;
extern char UARTMessageArray[101];
extern int UARTreceivelength;
extern union mem_add {
    float f;
    long i;
    char c[2];
}memloc;

extern union ptrmem_add {
    float* f;
    long* i;
    char c[2];
}ptrmemloc;

int32_t timessend = 0;
int32_t timelastsend = 0;



void mains_code(void) {
    //
    // Step 1. Initialize System Control:
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the DSP2833x_SysCtrl.c file.
    //
    InitSysCtrl();

    //
    // Step 2. Initialize GPIO:
    // This example function is found in the DSP2833x_Gpio.c file and
    // illustrates how to set the GPIO to it's default state.
    //
    // InitGpio();  // Skipped for this example

    //
    // Step 3. Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    //
    DINT;

    //
    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the DSP2833x_PieCtrl.c file.
    //
    InitPieCtrl();

    //
    // Disable CPU interrupts and clear all CPU interrupt flags:
    //
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in DSP2833x_DefaultIsr.c.
    // This function is found in DSP2833x_PieVect.c.
    //
    InitPieVectTable();

    //
    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this file.
    //
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TINT0 = &cpu_timer0_isr;
    PieVectTable.XINT13 = &cpu_timer1_isr;
    PieVectTable.TINT2 = &cpu_timer2_isr;
    PieVectTable.SCIRXINTA = &RXAINT_recv_ready;
    PieVectTable.SCITXINTA = &TXAINT_data_sent;
    PieVectTable.SPIRXINTA = &SPI_RXint;
    PieVectTable.SCIRXINTB = &RXBINT_recv_ready;
    PieVectTable.SCITXINTB = &TXBINT_data_sent;
    PieVectTable.SCIRXINTC = &RXCINT_recv_ready;
    PieVectTable.SCITXINTC = &TXCINT_data_sent;
    PieVectTable.rsvd12_6 = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    //
    // Step 4. Initialize the Device Peripheral. This function can be
    //         found in DSP2833x_CpuTimers.c
    //
    InitCpuTimers();   // For this example, only initialize the Cpu Timers

#if (CPU_FRQ_150MHZ)
    //
    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 150MHz CPU Freq, 1 second Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 150, 1000);
    ConfigCpuTimer(&CpuTimer1, 150, 1000000);
    ConfigCpuTimer(&CpuTimer2, 150, 1000000);
#endif

#if (CPU_FRQ_100MHZ)
    //
    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 100MHz CPU Freq, 1 second Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 100, 1000000);
    ConfigCpuTimer(&CpuTimer1, 100, 1000000);
    ConfigCpuTimer(&CpuTimer2, 100, 1000000);
#endif

    //
    // To ensure precise timing, use write-only instructions to write to the 
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers (in DSP2833x_CpuTimers.h), the
    // below settings must also be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0
    CpuTimer1Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0
    CpuTimer2Regs.TCR.all = 0x4000; //write-only instruction to set TSS bit = 0

    //
    // Step 5. User specific code, enable interrupts
    //

    init_serialSCIA(&SerialA,115200);
    init_serialSCIB(&SerialB,115200);
    init_serialSCIC(&SerialC,115200);

    init_PWMandDIR(1);
    init_PWMandDIR(2);
    init_PWMandDIR(3);
    init_PWMandDIR(4);
    init_PWMandDIR(5);
    //
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:
    //
    IER |= M_INT1;
    IER |= M_INT6;
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 8
    PieCtrlRegs.PIEIER12.bit.INTx6 = 1;

    EALLOW;  // set up LED GPIOs
    GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO30 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;
    GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;
    GpioCtrlRegs.GPBMUX1.bit.GPIO34 = 0;
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO34 = 1;

    GpioCtrlRegs.GPBMUX2.bit.GPIO60 = 0;
    GpioDataRegs.GPBSET.bit.GPIO60 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO60 = 1;

    GpioCtrlRegs.GPCMUX2.bit.GPIO85 = 0;
    GpioDataRegs.GPCSET.bit.GPIO85 = 1;
    GpioCtrlRegs.GPCDIR.bit.GPIO85 = 1;

    GpioCtrlRegs.GPBMUX2.bit.GPIO59 = 0;
    //GpioDataRegs.GPBCLEAR.bit.GPIO59 = 1;
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
    GpioCtrlRegs.GPBDIR.bit.GPIO59 = 1;

    // set up GPIO3 for amp enable or disable
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;
    GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;
    GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;

    EDIS;

    init_SPI();

    init_PWMandDIR(1);
    init_PWMandDIR(2);
    init_PWMandDIR(3);
    init_PWMandDIR(4);
    init_PWMandDIR(5);

    //
    // Enable global Interrupts and higher priority real-time debug events:
    //
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global realtime interrupt DBGM

    //
    // Step 6. IDLE loop. Just sit and loop forever (optional):
    //
    while(1) {
        if (UARTprint == 1){
            UARTprint = 0;
            printing();
            //            serial_printf(&SerialA, "%.2f %.2f,%.2f   \n\r",printtheta1motor*180/PI,printtheta2motor*180/PI,printtheta3motor*180/PI);

        }
        if (MatlabCommand == 1) {
            MatlabCommand = 0;

            if ('2' == UARTMessageArray[0]){

                memloc.c[1] = NULL;
                memloc.c[0] = ((UARTMessageArray[5]&0xFF)<<8);
                memloc.c[0] |= (UARTMessageArray[6]&0xFF);
                Main_memcount = memloc.i;
                ptrmemloc.c[1] = ((UARTMessageArray[1]&0xFF)<<8);
                ptrmemloc.c[1] |= (UARTMessageArray[2]&0xFF);
                ptrmemloc.c[0] = ((UARTMessageArray[3]&0xFF)<<8);
                ptrmemloc.c[0] |= (UARTMessageArray[4]&0xFF);
                Main_SendArray[0]='*';
                Main_SendArray[1]=3+Main_memcount;
                Main_SendArray[2]='3';

                serial_sendSCIC(&SerialC, Main_SendArray, 3);

                for (Main_i = 0; Main_i < Main_memcount;Main_i++){
                    Main_tempf = *ptrmemloc.f;
                    memloc.f = Main_tempf;
                    Main_SendArray2[0+Main_j*4] =  (memloc.c[0]&0xFF);
                    Main_SendArray2[1+Main_j*4] =  ((memloc.c[0]>>8)&0xFF);
                    Main_SendArray2[2+Main_j*4] =  (memloc.c[1]&0xFF);
                    Main_SendArray2[3+Main_j*4] =  ((memloc.c[1]>>8)&0xFF);
                    memloc.c[1] = ptrmemloc.c[1];
                    memloc.c[0] = ptrmemloc.c[0];
                    memloc.i+=2;  // was plus 4
                    ptrmemloc.c[1]=memloc.c[1];
                    ptrmemloc.c[0]=memloc.c[0];
                    Main_j++;
                    if (32 == Main_j){
                        memcpy(Main_SendArray,Main_SendArray2,128);
                        serial_sendSCIC(&SerialC, Main_SendArray, 128);
                        timessend++;
                        Main_j = 0;
                    }
                }
                if (Main_j != 0){
                    serial_sendSCIC(&SerialC, Main_SendArray2, (Main_memcount%32)*4);
                    timelastsend++;
                    Main_j = 0;
                }
                // Case '3' : Write float value to memory address (big-endian received address,
                //      little-endian received value)
            }else if ('3' == UARTMessageArray[0]){
                for (Main_i = 0; Main_i < (UARTreceivelength - 2)/8;Main_i++){

                    ptrmemloc.c[1] = ((UARTMessageArray[1+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[1] |= (UARTMessageArray[2+8*Main_i]&0xFF);
                    ptrmemloc.c[0] = ((UARTMessageArray[3+8*Main_i]&0xFF)<<8);
                    ptrmemloc.c[0] |= (UARTMessageArray[4+8*Main_i]&0xFF);

                    memloc.c[1] = ((UARTMessageArray[8+8*Main_i]&0xFF)<<8);
                    memloc.c[1] |= (UARTMessageArray[7+8*Main_i]&0xFF);
                    memloc.c[0] = ((UARTMessageArray[6+8*Main_i]&0xFF)<<8);
                    memloc.c[0] |= (UARTMessageArray[5+8*Main_i]&0xFF);

                    *ptrmemloc.i = memloc.i;

                }

            }
        }
    }
}

//
// cpu_timer0_isr - 
//
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    if (CpuTimer0.InterruptCount % 250 == 0) {
//        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }
    start_SPI();
    //
    // Acknowledge this interrupt to receive more interrupts from group 1
    //
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

//
// cpu_timer1_isr - 
//
__interrupt void cpu_timer1_isr(void)
{
    CpuTimer1.InterruptCount++;

    //
    // The CPU acknowledges the interrupt.
    //
    EDIS;
}

//
// cpu_timer2_isr -
//
__interrupt void cpu_timer2_isr(void)
{ 
    EALLOW;
    CpuTimer2.InterruptCount++;

    //
    // The CPU acknowledges the interrupt.
    //
    EDIS;
}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts



    // Insert SWI ISR Code here.......
    control();


    numSWIcalls++;

    DINT;

}

void releaseBrk(void){
    GpioDataRegs.GPBSET.bit.GPIO59 = 1;
}

void control(void) {

    updateData();


    // ---------- PD Control for Motor #1 ----------
    Main_e1 = Main_angle1_ref - Main_Enc1_rad;  // Compute Error
    Main_v1 = (Main_Enc1_rad - Main_Enc1_rad_old)/.001;
    Main_v1 = (Main_v1 +Main_v1old1 + Main_v1old2)/3.0;

    if(Main_e1 > Main_e1_thresh){
        Main_Kp1 = Main_Kp1_long;
    } else if(Main_e1 < -Main_e1_thresh) {
        Main_Kp1 = Main_Kp1_long;
    } else {
        Main_Kp1 = Main_Kp1_short;
    }

    Main_v1old2 = Main_v1old1;
    Main_v1old1 = Main_v1;
    Main_Enc1_rad_old = Main_Enc1_rad;

    // ---------- PD Control for Motor #2 ----------
    Main_e2 = Main_angle2_ref - Main_Enc2_rad;  // Compute Error
    Main_v2 = (Main_Enc2_rad - Main_Enc2_rad_old)/.001;
    Main_v2 = (Main_v2 +Main_v2old2 + Main_v2old2)/3.0; // Averaging filter

    if(Main_e2 > Main_e2_thresh){
        Main_Kp2 = Main_Kp2_long;
    } else if(Main_e2 < -Main_e2_thresh) {
        Main_Kp2 = Main_Kp2_long;
    } else {
        Main_Kp2 = Main_Kp2_short;
    }

    Main_v2old2 = Main_v2old1;
    Main_v2old1 = Main_v2;
    Main_Enc2_rad_old = Main_Enc2_rad;

    // ---------- PD Control for Motor #3 ----------
    Main_e3 = Main_angle3_ref - Main_Enc3_rad;  // Compute Error
    Main_v3 = (Main_Enc3_rad - Main_Enc3_rad_old)/.001;
    Main_v3 = (Main_v3 + Main_v3old1 + Main_v3old2)/3.0;

    if(Main_e3 > Main_e3_thresh){
        Main_Kp3 = Main_Kp3_long;
    } else if(Main_e3 < -Main_e3_thresh) {
        Main_Kp3 = Main_Kp3_long;
    } else {
        Main_Kp3 = Main_Kp3_short;
    }
    Main_v3old2 = Main_v3old1;
    Main_v3old1 = Main_v3;
    Main_Enc3_rad_old = Main_Enc3_rad;


    // ---------- PD Control for Motor #4 ----------
    Main_e4 = Main_angle4_ref - Main_Enc4_rad;  // Compute Error
    Main_v4 = (Main_Enc4_rad - Main_Enc4_rad_old)/.001;
    Main_v4 = (Main_v4 + Main_v4old1 + Main_v4old2)/3.0;

    if(Main_e4 > Main_e4_thresh){
        Main_Kp4 = Main_Kp4_long;
    } else if(Main_e4 < -Main_e4_thresh) {
        Main_Kp4 = Main_Kp4_long;
    } else {
        Main_Kp4 = Main_Kp4_short;
    }

    Main_v4old2 = Main_v4old1;
    Main_v4old1 = Main_v4;
    Main_Enc4_rad_old = Main_Enc4_rad;


    // ---------- PD Control for Motor #5 ----------
    Main_e5 = Main_angle5_ref - Main_Enc5_rad;  // Compute Error
    Main_v5 = (Main_Enc5_rad - Main_Enc5_rad_old)/.001;
    Main_v5 = (Main_v5 + Main_v5old1 + Main_v5old2)/3.0;

    if(Main_e5 > Main_e5_thresh){
        Main_Kp5 = Main_Kp5_long;
    } else if(Main_e5 < -Main_e5_thresh) {
        Main_Kp5 = Main_Kp5_long;
    } else {
        Main_Kp5 = Main_Kp5_short;
    }

    Main_v5old2 = Main_v5old1;
    Main_v5old1 = Main_v5;
    Main_Enc5_rad_old = Main_Enc5_rad;

    Main_u1 = Main_Kp1*(Main_angle1_ref - Main_Enc1_rad) - Main_Kd1*Main_v1;
    Main_u2 = Main_Kp2*(Main_angle2_ref - Main_Enc2_rad) - Main_Kd2*Main_v2;
    Main_u3 = Main_Kp3*(Main_angle3_ref - Main_Enc3_rad) - Main_Kd3*Main_v3;
    Main_u4 = Main_Kp4*(Main_angle4_ref - Main_Enc4_rad) - Main_Kd4*Main_v4;
    Main_u5 = Main_Kp5*(Main_angle5_ref - Main_Enc5_rad) - Main_Kd5*Main_v5;



    if(Main_timeint == 4000){
        releaseBrk();
    }
    if (Main_timeint < 4100){
        Main_u1 = 0;
        Main_u2 = 0;
        Main_u3 = 0;
        Main_u4 = 0;
        Main_u5 = 0;
    } else {

        if ( fabs(Main_angle5_ref - Main_Enc5_rad) > 0.11 ) {
            shutoffEnc5error  = 1;
        }

        lab(Main_Enc1_rad,Main_Enc2_rad,Main_Enc3_rad,&Main_labtau1,&Main_labtau2,&Main_labtau3, Main_safetymode);

        Main_thetaDH1 = Main_Enc1_rad;
        Main_thetaDH2 = Main_Enc2_rad - PI*0.5;
        Main_thetaDH3 = Main_Enc3_rad - Main_Enc2_rad + PI*0.5;

        // forward kinematics
        //Main_cosq1 = cos(Main_Enc1_rad);
        //Main_sinq1 = sin(Main_Enc1_rad);
        Main_cosq2 = cos(Main_Enc2_rad);
        //Main_sinq2 = sin(Main_Enc2_rad);
        //Main_cosq3 = cos(Main_Enc3_rad);
        Main_sinq3 = sin(Main_Enc3_rad);
        //Main_xk = 10*Main_cosq1*(Main_cosq3 + Main_sinq2);
        //Main_yk = 10*Main_sinq1*(Main_cosq3 + Main_sinq2);
        Main_zk = 10*(1+ Main_cosq2 - Main_sinq3);

        if ( (Main_thetaDH1 > DHTHETA1_MAX) ||
                (Main_thetaDH1 < DHTHETA1_MIN) ||
                (Main_thetaDH2 > DHTHETA2_MAX) ||
                (Main_thetaDH2 < DHTHETA2_MIN) ||
                (Main_thetaDH3 > DHTHETA3_MAX) ||
                (Main_thetaDH3 < DHTHETA3_MIN) ||
                (Main_zk < Z_MIN) ) {
            Main_safetymode = 1;
        } else {
            Main_safetymode = 0;
            Main_angle1_ref = Main_Enc1_rad;
            Main_angle2_ref = Main_Enc2_rad;
            Main_angle3_ref = Main_Enc3_rad;
            Main_angle4_ref = 0;
            Main_angle5_ref = 0;

            // Angles are in range so use lab control effort
            Main_u1 = Main_labtau1;
            Main_u2 = Main_labtau2;
            Main_u3 = Main_labtau3;

        }
        if (Main_safetymode == 1) {
            if (fabs(Main_angle1_ref-Main_Enc1_rad) > 0.15) {
                Main_angle1_ref = Main_Enc1_rad;
            }
            if (fabs(Main_angle2_ref-Main_Enc2_rad) > 0.15) {
                Main_angle2_ref = Main_Enc2_rad;
            }
            if (fabs(Main_angle3_ref-Main_Enc3_rad) > 0.15) {
                Main_angle3_ref = Main_Enc3_rad;
            }
        }

    }

//    if (Main_u1 > 2.5) Main_u1 = 2.5;
//    if (Main_u1 < -2.5) Main_u1 = -2.5;
//    if (Main_u2 > 2.5) Main_u2 = 2.5;
//    if (Main_u2 < -2.5) Main_u2 = -2.5;
//    if (Main_u3 > 2.5) Main_u3 = 2.5;
//    if (Main_u3 < -2.5) Main_u3 = -2.5;
//    if (Main_u4 > 2.5) Main_u4 = 2.5;
//    if (Main_u4 < -2.5) Main_u4 = -2.5;
//    if (Main_u5 > 2.5) Main_u5 = 2.5;
//    if (Main_u5 < -2.5) Main_u5 = -2.5;

    if (Main_u1 > 5) Main_u1 = 5;
    if (Main_u1 < -5) Main_u1 = -5;
    if (Main_u2 > 5) Main_u2 = 5;
    if (Main_u2 < -5) Main_u2 = -5;
    if (Main_u3 > 5) Main_u3 = 5;
    if (Main_u3 < -5) Main_u3 = -5;
    if (Main_u4 > 5) Main_u4 = 5;
    if (Main_u4 < -5) Main_u4 = -5;
    if (Main_u5 > 5) Main_u5 = 5;
    if (Main_u5 < -5) Main_u5 = -5;

    Main_u1Integral += fabs((Main_u1 + Main_u1_old)*0.0005);
    Main_u2Integral += fabs((Main_u2 + Main_u2_old)*0.0005);
    Main_u3Integral += fabs((Main_u3 + Main_u3_old)*0.0005);

    Main_IntegralSum = Main_u1Integral + Main_u2Integral + Main_u3Integral;

    Main_u1_old = Main_u1;
    Main_u2_old = Main_u2;
    Main_u3_old = Main_u3;

    if (shutoffEnc5error == 0) {
        // 6/3/2016 PWM to be sent on SPI to 2nd MC.
        PWMandDIR_out(1,Main_u1);
        PWMandDIR_out(2,-Main_u2);
        PWMandDIR_out(3,Main_u3);
        PWMandDIR_out(4,Main_u4);
        PWMandDIR_out(5,-Main_u5);

        sendPWM(Main_u1,-Main_u2,Main_u3,Main_u4,-Main_u5);
    } else {
        PWMandDIR_out(1,0);
        PWMandDIR_out(2,0);
        PWMandDIR_out(3,0);
        PWMandDIR_out(4,0);
        PWMandDIR_out(5,0);

        sendPWM(0,0,0,0,0);

    }

    Main_timeint++;

}

void updateData(void) {

    //    Main_value_enc1 = EQEP_read(&Main_enc1);
    //    Main_value_enc2 = EQEP_read(&Main_enc2);

    Main_Enc1_rad = ((float) (SPIenc1_reading)*(PI/(72000.0*2)));
    Main_Enc2_rad = ((float) (SPIenc2_reading)*(-PI/(72000.0*2))) + offset_Enc2_rad;
    Main_Enc3_rad = ((float) (SPIenc3_reading)*(PI/(72000.0*2))) + offset_Enc3_rad;
    Main_Enc4_rad = ((float) (SPIenc4_reading)*(PI/(8000.0*2)));
    Main_Enc5_rad = ((float) (SPIenc5_reading)*(-PI/(8000.0*2)));


}

//
// End of File
//

