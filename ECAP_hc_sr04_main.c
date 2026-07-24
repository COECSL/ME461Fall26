// Included Files
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <math.h>
#include <limits.h>
#include "F28x_Project.h"
#include "driverlib.h"
#include "device.h"
#include "f28379dSerial.h"

//------------------------------------------------------

#define TIMEBASE 0.005 // 1.0/200

// VCC  - +5V
// GND  - GND
// TRIG - GPIO0
// ECHO - GPIO19

uint32_t echo_count    = 0;
float echo_duration    = 0;
float echo_distance    = 0;
uint16_t trigger_count = 0;
uint16_t trigger_state = 0;

// This function should be called every 1ms
void hc_sr04_trigger(void) {

    if (trigger_count < 2) {

        if (trigger_state == 0) {
            // last for 2ms
            GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;
            trigger_state = 1;
        }
        trigger_count++;

    } else if ( (trigger_count >= 2) && (trigger_count < 11) ) {

        // last for 10ms
        if (trigger_state == 1) {
            GpioDataRegs.GPASET.bit.GPIO0 = 1;
            trigger_state = 2;
        }
        trigger_count++;

    } else {

        if (trigger_state == 2) {
            trigger_count = 0;
            trigger_state = 0;
        }

    }

}

// InitECapture - Initialize ECAP1 configurations
void InitECapture() {

    EALLOW;
    DevCfgRegs.SOFTPRES3.bit.ECAP1 = 1;     // eCAP1 is reset
    DevCfgRegs.SOFTPRES3.bit.ECAP1 = 0;     // eCAP1 is released from reset
    EDIS;

    ECap1Regs.ECEINT.all           = 0;     // Disable all eCAP interrupts
    ECap1Regs.ECCTL1.bit.CAPLDEN   = 0;     // Disabled loading of capture results
    ECap1Regs.ECCTL2.bit.TSCTRSTOP = 0;     // Stop the counter

    ECap1Regs.TSCTR  = 0;                   // Clear the counter
    ECap1Regs.CTRPHS = 0;                   // Clear the counter phase register

    // ECAP control register 2
    ECap1Regs.ECCTL2.all = 0x0096;
    // bit 15-11     00000:  reserved
    // bit 10        0:      APWMPOL, don't care
    // bit 9         0:      CAP/APWM, 0 = capture mode, 1 = APWM mode
    // bit 8         0:      SWSYNC, 0 = no action (no s/w synch)
    // bit 7-6       10:     SYNCO_SEL, 10 = disable sync out signal
    // bit 5         0:      SYNCI_EN, 0 = disable Sync-In
    // bit 4         1:      TSCTRSTOP, 1 = enable counter
    // bit 3         0:      RE-ARM, 0 = don't re-arm, 1 = re-arm
    // bit 2-1       11:     STOP_WRAP, 11 = wrap after 4 captures
    // bit 0         0:      CONT/ONESHT, 0 = continuous mode

    // ECAP control register 1
    ECap1Regs.ECCTL1.all = 0xC144;
    // bit 15-14     11:     FREE/SOFT, 11 = ignore emulation suspend
    // bit 13-9      00000:  PRESCALE, 00000 = divide by 1
    // bit 8         1:      CAPLDEN, 1 = enable capture results load
    // bit 7         0:      CTRRST4, 0 = do not reset counter on CAP4 event
    // bit 6         1:      CAP4POL, 0 = rising edge, 1 = falling edge
    // bit 5         0:      CTRRST3, 0 = do not reset counter on CAP3 event
    // bit 4         0:      CAP3POL, 0 = rising edge, 1 = falling edge
    // bit 3         0:      CTRRST2, 0 = do not reset counter on CAP2 event
    // bit 2         1:      CAP2POL, 0 = rising edge, 1 = falling edge
    // bit 1         0:      CTRRST1, 0 = do not reset counter on CAP1 event
    // bit 0         0:      CAP1POL, 0 = rising edge, 1 = falling edge

    // Enable desired eCAP interrupts
    ECap1Regs.ECEINT.all = 0x0008;
    // bit 15-8      0's:    reserved
    // bit 7         0:      CTR=CMP, 0 = compare interrupt disabled
    // bit 6         0:      CTR=PRD, 0 = period interrupt disabled
    // bit 5         0:      CTROVF, 0 = overflow interrupt disabled
    // bit 4         0:      CEVT4, 0 = event 4 interrupt disabled
    // bit 3         1:      CEVT3, 1 = event 3 interrupt enabled
    // bit 2         0:      CEVT2, 0 = event 2 interrupt disabled
    // bit 1         0:      CEVT1, 0 = event 1 interrupt disabled
    // bit 0         0:      reserved

}

__interrupt void ecap1_isr(void);

//------------------------------------------------------

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398

// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);

void serialRXA(serial_t *s, char data);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls = 0;
uint32_t numRXA = 0;
uint16_t UARTPrint = 0;

void setup_led_GPIO(void);

void main(void) {

    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    setup_led_GPIO();

    //---------------------------------------------------------

    // Trigger pin for HC-SR04
    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;

    // Echo pin for HC-SR04
    EALLOW;
    InputXbarRegs.INPUT7SELECT = 19; // Set eCAP1 source to GPIO-pin
    EDIS;
    GPIO_SetupPinOptions(19, GPIO_INPUT, GPIO_ASYNC);

    InitECapture();

    //---------------------------------------------------------

    // Clear all interrupts and initialize PIE vector table:
    // Disable CPU interrupts
    DINT;

    // Initialize the PIE control registers to their default state.
    // The default state is all PIE interrupts disabled and flags
    // are cleared.
    // This function is found in the F2837xD_PieCtrl.c file.
    InitPieCtrl();

    // Disable CPU interrupts and clear all CPU interrupt flags:
    IER = 0x0000;
    IFR = 0x0000;

    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    // This will populate the entire table, even if the interrupt
    // is not used in this example.  This is useful for debug purposes.
    // The shell ISR routines are found in F2837xD_DefaultIsr.c.
    // This function is found in F2837xD_PieVect.c.
    InitPieVectTable();

    // Interrupts that are used in this example are re-mapped to
    // ISR functions found within this project
    EALLOW;  // This is needed to write to EALLOW protected registers
    PieVectTable.TIMER0_INT = &cpu_timer0_isr;
    PieVectTable.TIMER1_INT = &cpu_timer1_isr;
    PieVectTable.TIMER2_INT = &cpu_timer2_isr;

    PieVectTable.SCIA_RX_INT = &RXAINT_recv_ready;
    PieVectTable.SCIC_RX_INT = &RXCINT_recv_ready;
    PieVectTable.SCID_RX_INT = &RXDINT_recv_ready;
    PieVectTable.SCIA_TX_INT = &TXAINT_data_sent;
    PieVectTable.SCIC_TX_INT = &TXCINT_data_sent;
    PieVectTable.SCID_TX_INT = &TXDINT_data_sent;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;

    PieVectTable.ECAP1_INT = &ecap1_isr;

    EDIS;    // This is needed to disable write to EALLOW protected registers

    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
//    ConfigCpuTimer(&CpuTimer1, 200, 20000);
//    ConfigCpuTimer(&CpuTimer2, 200, 40000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
//    CpuTimer1Regs.TCR.all = 0x4000;
//    CpuTimer2Regs.TCR.all = 0x4000;

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.
    IER |= M_INT1;
    IER |= M_INT4;  // Enable CPU INT4 which is connected to ECAP1-4 INT
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;

    // Enable eCAP INT1 in the PIE: Group 4 interrupt 1
    PieCtrlRegs.PIEIER4.bit.INTx1 = 1;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;

    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM


    // IDLE loop. Just sit and loop forever (optional):
    while(1) {

    }

}


// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    // Insert SWI ISR Code here.......

    numSWIcalls++;
    UARTPrint = 1;  // Signal main while loop to print to Terminal

    DINT;

}


// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void) {

    CpuTimer0.InterruptCount++;

    numTimer0calls++;

//    if (numTimer0calls%50 == 0) {
//        PieCtrlRegs.PIEIFR12.bit.INTx9 = 1;  // Manually cause the interrupt for the SWI
//    }


    // Trigger HC-SR04
    hc_sr04_trigger();


    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}


// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void) {

//    // Blink LaunchPad Red LED
//    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;

    CpuTimer1.InterruptCount++;

}


// cpu_timer2_isr CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void) {

//    // Blink LaunchPad Blue LED
//    GpioDataRegs.GPATOGGLE.bit.GPIO31 = 1;

    CpuTimer2.InterruptCount++;

}

__interrupt void ecap1_isr(void) {

    PieCtrlRegs.PIEACK.all    = PIEACK_GROUP4;  // Must acknowledge the PIE group

    ECap1Regs.ECCLR.bit.INT   = 1;              // Clear the ECAP1 interrupt flag
    ECap1Regs.ECCLR.bit.CEVT3 = 1;              // Clear the CEVT3 flag

    // The eCAP is running at the full 200 MHz of the device.
    // Captured values reflect this time base.

    // Compute the PWM duty period (rising edge to falling edge)
    echo_count = (int32)ECap1Regs.CAP2 - (int32)ECap1Regs.CAP1;

    echo_duration = (float)(TIMEBASE * echo_count); // in us

    echo_distance = echo_duration * 0.034 / 2.0; // cm

}

// This function is called each time a char is recieved over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
}

void setup_led_GPIO(void) {

    // Blue LED on LuanchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

}




























