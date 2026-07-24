//#############################################################################
// FILE:   labstarter_main.c
//
// TITLE:  Lab Starter
//#############################################################################


//https://www.hackster.io/evanah2/comparing-direction-of-travel-methods-me461-uiuc-f14c92

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

#define PI          3.1415926535897932384626433832795
#define TWOPI       6.283185307179586476925286766559
#define HALFPI      1.5707963267948966192313216916398


// Interrupt Service Routines predefinition
__interrupt void cpu_timer0_isr(void);
__interrupt void cpu_timer1_isr(void);
__interrupt void cpu_timer2_isr(void);
__interrupt void SWI_isr(void);
__interrupt void SPIB_isr(void);

void serialRXA(serial_t *s, char data);
void setupSpib(void);
void initEPwm6AB(void);
void setEPWM6A(float uRight);
void setEPWM6B(float uLeft);

float readEncLeft(void);
float readEncRight(void);
void init_eQEPs(void);

// Count variables
uint32_t numTimer0calls = 0;
uint32_t numSWIcalls    = 0;
uint32_t numRXA         = 0;
uint16_t UARTPrint      = 0;

float accelx = 0;
float accely = 0;
float accelz = 0;

float gyrox  = 0;
float gyroy  = 0;
float gyroz  = 0;

float mx     = 0;
float my     = 0;
float mz     = 0;

float accelx_offset = 0;
float accely_offset = 0;
float accelz_offset = 0;

float gyrox_offset  = 0;
float gyroy_offset  = 0;
float gyroz_offset  = 0;

float accelxSide = 1.502;      //accel x offset for when starting on the side

float accelxSideStart = 1.502;

float accelzSide = -0.76;

float gravitySide = 0.70;       //gravity offset for accely when starting on the side.


//char buffer[100];
int16 IMU_data[9];

//float offset = 0;
uint16_t count=0;
uint16_t newdata=0;
uint16_t read=0;
uint16_t calc_offset=0;

uint16_t MPU_readbacktest[34];

// Joey added constants
uint16_t XG_OFFSET_H        = 0x1300;
uint16_t XG_OFFSET_L        = 0x1400;
uint16_t YG_OFFSET_H        = 0x1500;
uint16_t YG_OFFSET_L        = 0x1600;
uint16_t ZG_OFFSET_H        = 0x1700;
uint16_t ZG_OFFSET_L        = 0x1800;

uint16_t SMPLRT_DIV         = 0x1900;

uint16_t CONFIG             = 0x1A00;
uint16_t GYRO_CONFIG        = 0x1B00;
uint16_t ACCEL_CONFIG       = 0x1C00;
uint16_t ACCEL_CONFIG_2     = 0x1D00;

uint16_t LP_ACCEL_ODR       = 0x1E00;
uint16_t WOM_THR            = 0x1F00;
uint16_t FIFO_EN            = 0x2300;

uint16_t I2C_MST_CTRL       = 0x2400;
uint16_t I2C_SLV0_ADDR      = 0x2500;
uint16_t I2C_SLV0_REG       = 0x2600;
uint16_t I2C_SLV0_CTRL      = 0x2700;
uint16_t I2C_SLV1_ADDR      = 0x2800;
uint16_t I2C_SLV1_REG       = 0x2900;
uint16_t I2C_SLV1_CTRL      = 0x2A00;

uint16_t INT_ENABLE         = 0x3800;
uint16_t INT_STATUS         = 0x3A00;

uint16_t I2C_SLV1_DO        = 0x6400;
uint16_t I2C_MST_DELAY_CTRL = 0x6700;

uint16_t USER_CTRL          = 0x6A00;
uint16_t PWR_MGMT_1         = 0x6B00;
uint16_t WHO_AM_I           = 0x7500;

uint16_t XA_OFFSET_H        = 0x7700;
uint16_t XA_OFFSET_L        = 0x7800;
uint16_t YA_OFFSET_H        = 0x7A00;
uint16_t YA_OFFSET_L        = 0x7B00;
uint16_t ZA_OFFSET_H        = 0x7D00;
uint16_t ZA_OFFSET_L        = 0x7E00;



uint16_t temp=0;
uint16_t senddata = 0;

int16_t readdata[25];  // received data

uint16_t acc_config2 = 0;

float x_A_offs_add = .100;
float y_A_offs_add = -.07;
float z_A_offs_add = .070;

float x_G_offs = 1;
float y_G_offs = 6;
float z_G_offs = 1;

float compass_angle = 0.0;

float max_mx = 53;
float min_mx = -92;
float max_my = 73;
float min_my = -86;
float max_mz = 273;
float min_mz = 125;

float mx_offset = (53 + -92)/2.0; //(max_mx + min_mx)/2.0
float my_offset = (73 + -86)/2.0; // (max_my + min_my)/2.0
float mz_offset = (273 + 125)/2.0; // (max_mz + min_mz)/2.0

float wheelLeft = 0;
float wheelRight = 0;
float uLeft = 5.0;
float uRight = 5.0;
float dutyValue = 0;
float travelLeft = 0;
float travelRight = 0;

float XLeft_K = 0;
float XLeft_K_1 = 0;
float VLeftK = 0;

float XRight_K = 0;
float XRight_K_1 = 0;
float VRightK = 0;

float kp = 3;
float ki = 15;

float LI_K = 0;
float LI_K_1 = 0;
float RI_K = 0;
float RI_K_1 = 0;
float LE_K = 0;
float LE_K_1 = 0;
float RE_K = 0;
float RE_K_1 = 0;
float LU_K = 0;
float RU_K = 0;
float Vref = 0.5;

float eturn = 0;
float kturn = 3;
float turn = 0;

float gyroz_1 = 0;
float gyroZint = 0;
float gyroZint_1 = 0;

float R = 1.275; //wheel radius in inches
float W = 7.35;  //base width in inches
float phi = 0;

float phi_array[500];
float compass_angle_array[500];
float gyroZint_array[500];
int16_t spibcount = 0;

int16_t revcountphi = 0;
int16_t revcountgyro = 0;
float gyroZintdisplay = 0;
int16_t doneCal = 0;

void main(void)
{
    // PLL, WatchDog, enable Peripheral Clocks
    // This example function is found in the F2837xD_SysCtrl.c file.
    InitSysCtrl();

    InitGpio();

    // Blue LED on LaunchPad
    GPIO_SetupPinMux(31, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(31, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO31 = 1;

    // Red LED on LaunchPad
    GPIO_SetupPinMux(34, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(34, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO34 = 1;

    // LED1 and PWM Pin
    GPIO_SetupPinMux(22, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(22, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO22 = 1;

    // LED2
    GPIO_SetupPinMux(52, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(52, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO52 = 1;

    // LED3
    GPIO_SetupPinMux(67, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(67, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO67 = 1;

    // LED4
    GPIO_SetupPinMux(94, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(94, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO94 = 1;

    // LED5
    GPIO_SetupPinMux(95, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(95, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCCLEAR.bit.GPIO95 = 1;

    // LED6
    GPIO_SetupPinMux(97, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(97, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO97 = 1;

    // LED7
    GPIO_SetupPinMux(111, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(111, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPDCLEAR.bit.GPIO111 = 1;

    // LED8
    GPIO_SetupPinMux(130, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(130, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO130 = 1;

    // LED9
    GPIO_SetupPinMux(131, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(131, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO131 = 1;

    // LED10
    GPIO_SetupPinMux(4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;

    // LED11
    GPIO_SetupPinMux(5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(5, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;

    // LED12
    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;

    // LED13
    GPIO_SetupPinMux(7, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(7, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;

    // LED14
    GPIO_SetupPinMux(8, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(8, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;

    // LED15
    GPIO_SetupPinMux(9, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(9, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;

    // LED16
    GPIO_SetupPinMux(24, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(24, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;

    // LED17
    GPIO_SetupPinMux(25, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(25, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

    // LED18
    GPIO_SetupPinMux(26, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(26, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO26 = 1;

    // LED19
    GPIO_SetupPinMux(27, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(27, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPACLEAR.bit.GPIO27 = 1;

    // LED20
    GPIO_SetupPinMux(60, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(60, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO60 = 1;

    // LED21
    GPIO_SetupPinMux(61, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(61, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBCLEAR.bit.GPIO61 = 1;

    // LED22
    GPIO_SetupPinMux(157, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(157, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO157 = 1;

    // LED23
    GPIO_SetupPinMux(158, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(158, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPECLEAR.bit.GPIO158 = 1;

    //DRV8874 #1 DIR  Direction
    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO29 = 1;

    //DRV8874 #2 DIR  Direction
    GPIO_SetupPinMux(32, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(32, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;

    //MPU9250  CS  Chip Select
    GPIO_SetupPinMux(66, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(66, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;

    //DAN777  CS  Chip Select
    GPIO_SetupPinMux(2, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(2, GPIO_OUTPUT, GPIO_PUSHPULL);
    GpioDataRegs.GPASET.bit.GPIO2 = 1;

    //PushButton 1
    GPIO_SetupPinMux(122, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(122, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 2
    GPIO_SetupPinMux(123, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(123, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 3
    GPIO_SetupPinMux(124, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(124, GPIO_INPUT, GPIO_PULLUP);

    //PushButton 4
    GPIO_SetupPinMux(125, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(125, GPIO_INPUT, GPIO_PULLUP);

    //EPWM6A - Right Motor
    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 1);

    //EPWM6B - Left Motor
    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 1);

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
    PieVectTable.SPIB_RX_INT = &SPIB_isr;

    PieVectTable.EMIF_ERROR_INT = &SWI_isr;
    EDIS;    // This is needed to disable write to EALLOW protected registers

    // DC Motor Setup
    EPwm6Regs.TBCTL.bit.FREE_SOFT = 2;   // Free run
    EPwm6Regs.TBCTL.bit.CLKDIV = 0;    //  Divide by 1
    EPwm6Regs.TBCTL.bit.CTRMODE = 0;  // Count up mode
    EPwm6Regs.TBCTL.bit.PHSEN = 0; // Disable phase. Prof. Block says we need this
    EPwm6Regs.TBCTR = 0; // Start timers at zero
    EPwm6Regs.TBPRD = 1000; // 20kHz Period = 50us => 50us*50MHz = 2500 // 1000 for E3
    EPwm6Regs.CMPA.bit.CMPA = 0; // Duty cycle at (0/2500) * 100 = 0%
    EPwm6Regs.CMPB.bit.CMPB = 0; // Duty cycle at (0/2500) * 100 = 0%
    EPwm6Regs.AQCTLA.bit.CAU = 1; // Clears / grounds
    EPwm6Regs.AQCTLA.bit.ZRO = 2; // Sets high
    EPwm6Regs.AQCTLB.bit.CBU = 1; // Clears / grounds
    EPwm6Regs.AQCTLB.bit.ZRO = 2; // Sets high
    EPwm6Regs.TBPHS.bit.TBPHS = 0; // Being careful
    // Initialize the CpuTimers Device Peripheral. This function can be
    // found in F2837xD_CpuTimers.c
    InitCpuTimers();

    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 200MHz CPU Freq, 1 second Period (in uSeconds)
    ConfigCpuTimer(&CpuTimer0, 200, 1000);
    ConfigCpuTimer(&CpuTimer1, 200, 4000);
    ConfigCpuTimer(&CpuTimer2, 200, 1000);

    // Enable CpuTimer Interrupt bit TIE
    CpuTimer0Regs.TCR.all = 0x4000;
    CpuTimer1Regs.TCR.all = 0x4000;
    CpuTimer2Regs.TCR.all = 0x4000;

    // Enable eQEP
    init_eQEPs();

    GPIO_SetupPinMux(63, GPIO_MUX_CPU1, 15);  //MPU-9250 Data Input
    GPIO_SetupPinMux(64, GPIO_MUX_CPU1, 15);  //MPU-9250 Data Output
    GPIO_SetupPinMux(65, GPIO_MUX_CPU1, 15);  //MPU-9250 Clock

    EALLOW;
    GpioCtrlRegs.GPBPUD.bit.GPIO63 = 0;  // Enable Pull-ups
    GpioCtrlRegs.GPCPUD.bit.GPIO64 = 0;
    GpioCtrlRegs.GPCPUD.bit.GPIO65 = 0;
    GpioCtrlRegs.GPBQSEL2.bit.GPIO63 = 3; // Set prequalifier for 63,64,65
    GpioCtrlRegs.GPCQSEL1.bit.GPIO64 = 3;
    GpioCtrlRegs.GPCQSEL1.bit.GPIO65 = 3;
    EDIS;

    setupSpib(); // Init MPU9250

    init_serial(&SerialA,115200,serialRXA);
//    init_serial(&SerialC,115200,serialRXC);
//    init_serial(&SerialD,115200,serialRXD);

    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2:  int 12 is for the SWI.  
    IER |= M_INT1;
    IER |= M_INT8;  // SCIC SCID
    IER |= M_INT9;  // SCIA
    IER |= M_INT12;
    IER |= M_INT13;
    IER |= M_INT14;
    IER |= M_INT6;

    // Enable TINT0 in the PIE: Group 1 interrupt 7
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    // Enable SWI in the PIE: Group 12 interrupt 9
    PieCtrlRegs.PIEIER12.bit.INTx9 = 1;

    PieCtrlRegs.PIEIER6.bit.INTx3=1;// Enable PIE Group 6, INT 3 (spib_rx)


    // Enable global Interrupts and higher priority real-time debug events
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

    
    // IDLE loop. Just sit and loop forever (optional):
    while(1)
    {
        if (UARTPrint == 1 ) {
            //serial_printf(&SerialA,"X = %.1f Y = %.1f Z = %.1f Ang = %.1f turn %.3f\r\n", mx, my, mz, compass_angle, turn);
            serial_printf(&SerialA,"gyroZint = %.1f phi = %.1f Compass Angle = %.1f Revolutions = %i \r\n", gyroZintdisplay, phi, compass_angle, revcountphi);
            UARTPrint = 0;
        }
    }
}

__interrupt void SPIB_isr(void){


    GpioDataRegs.GPCSET.bit.GPIO66 = 1;                 //Pull CS high as done R/W

    uint16_t i;

    for (i=0; i<14; i++) {
        readdata[i] = SpibRegs.SPIRXBUF; // readdata[0] is garbage, readdata[8] is temperature
    }

    IMU_data[0] = readdata[1];
    IMU_data[1] = readdata[2];
    IMU_data[2] = readdata[3];
    IMU_data[3] = readdata[5];
    IMU_data[4] = readdata[6];
    IMU_data[5] = readdata[7];

    IMU_data[6] = readdata[9];
    IMU_data[7] = readdata[10];
    IMU_data[8] = readdata[11];

    accelx = (((float)(IMU_data[0]))*4.0/32767.0)-x_A_offs_add;
    accely = (((float)(IMU_data[1]))*4.0/32767.0)-y_A_offs_add;
    accelz = (((float)(IMU_data[2]))*4.0/32767.0)-z_A_offs_add;
    gyrox  = (((float)(IMU_data[3]))*250.0/32767.0)-x_G_offs;  // deg/s
    gyroy  = (((float)(IMU_data[4]))*250.0/32767.0)-y_G_offs;  // deg/s
    gyroz  = (((float)(IMU_data[5]))*250.0/32767.0)-z_G_offs;  // deg/s
    mx     = (((float)(IMU_data[6]))*1.0/1.0);
    my     = (((float)(IMU_data[7]))*1.0/1.0);
    mz     = (((float)(IMU_data[8]))*1.0/1.0);

    gyroZint = gyroZint_1 + ((0.001)*(gyroz + gyroz_1)/2); //SPIB_ISR running every 1 ms
    gyroZintdisplay = gyroZint - (360*revcountgyro);  //Keeps display within -360 to 360 degrees

    if(gyroZintdisplay > 360) //If it completes a revolution, subtract a revolution to keep display within -360 to 360 degrees
    {
        revcountgyro++;
    }
    if(gyroZintdisplay < -360)
    {
        revcountgyro--;
    }
    gyroz_1 = gyroz; //Fill the old value
    gyroZint_1 = gyroZint; //Fill the old value

    mx = mx - mx_offset;   //Add in the offset from calibration
    my = my - my_offset;   //Add in the offset from calibration
    //mz = mz - mz_offset; //Add in the offset from calibration

    phi = ((R/W)*(wheelRight - wheelLeft)*(180/PI) - (360*revcountphi));

    if(phi > 360) //If it completes a revolution, subtract a revolution to keep display within -360 to 360 degrees
    {
        revcountphi++;
    }
    if(phi < -360) //Also keeps track of revolutions to display
    {
        revcountphi--;
    }

    //Use arctan to get compass angle
    if(my < 0) {
        compass_angle = 270.0-(((float)atan((mx/my)))*180.0/PI);
    }
    else if(my > 0) {
        compass_angle = 90.0-(((float)atan((mx/my)))*180.0/PI);
    }
    else if((my == 0) && (mx < 0)) {
        compass_angle = 180.0;
    }
    if((my == 0) && (mx > 0)) {
        compass_angle = 0.0;
    }

//    Fill arrays of data
//    if(spibcount<500){
//        compass_angle_array[spibcount] = compass_angle;
//        phi_array[spibcount] = phi;
//        gyroZint_array[spibcount] = gyroZint;
//    }

    //This code calibrates the gyro relative to wherever the cart is being used
    if(calc_offset == 0)
    {}
    if(calc_offset==1){
        accelx_offset+=accelx;accely_offset+=accely;accelz_offset+=accelz;
        gyrox_offset+=gyrox;gyroy_offset+=gyroy;gyroz_offset+=gyroz;
    }
    else if(calc_offset==2){
        accelx_offset/=3000.0;accely_offset/=3000.0;accelz_offset/=3000.0;
        gyrox_offset/=3000.0;gyroy_offset/=3000.0;gyroz_offset/=3000.0;
        calc_offset = 3;
    }
    else if(calc_offset==3){
        doneCal = 1;
        accelx -=(accelx_offset-accelxSide);accely -=(accely_offset-gravitySide);accelz -=(accelz_offset-accelzSide);  //different offsets when start on side
        gyrox -= gyrox_offset;gyroy -= gyroy_offset;gyroz -= gyroz_offset;
    }
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR=1;  // Clear Overflow flag
    SpibRegs.SPIFFRX.bit.RXFFINTCLR=1;  // Clear Interrupt flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP6;

    spibcount++;

}

// SWI_isr,  Using this interrupt as a Software started interrupt
__interrupt void SWI_isr(void) {

    // These three lines of code allow SWI_isr, to be interrupted by other interrupt functions
    // making it lower priority than all other Hardware interrupts.
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP12;
    asm("       NOP");                    // Wait one cycle
    EINT;                                 // Clear INTM to enable interrupts

    numSWIcalls++;

    DINT;
}

// cpu_timer0_isr - CPU Timer0 ISR
__interrupt void cpu_timer0_isr(void)
{
    CpuTimer0.InterruptCount++;

    numTimer0calls++;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// cpu_timer1_isr - CPU Timer1 ISR
__interrupt void cpu_timer1_isr(void)
{
    wheelLeft = readEncLeft();
    wheelRight = -1* readEncRight(); //Right wheel is negative
    travelLeft = wheelLeft / 9.44; //Radians to feet
    travelRight = wheelRight / 9.44; //Radians to feet

    XLeft_K_1 = XLeft_K;  // Fill previous position
    XLeft_K = travelLeft;  //Grab new position
    VLeftK = (XLeft_K - XLeft_K_1 )/ (0.004); //Use the positions to calculate velocity

    XRight_K_1 = XRight_K;  // Fill previous position
    XRight_K = travelRight;  //Grab new position
    VRightK = (XRight_K - XRight_K_1 )/ (0.004); //Use the positions to calculate velocity

    //Turn controls
    eturn = turn + (VLeftK - VRightK);

    LE_K_1 = LE_K;
    RE_K_1 = RE_K;
    LE_K = Vref - VLeftK - (kturn*eturn);
    RE_K = Vref - VRightK + (kturn*eturn);

    //I controls
    LI_K_1 = LI_K;
    RI_K_1 = RI_K;
    LI_K = LI_K_1 +(0.004)*((LE_K_1 + LE_K)/2);
    RI_K = RI_K_1 +(0.004)*((RE_K_1 + RE_K)/2);

    //U controls
    LU_K = kp*LE_K + ki*LI_K;
    RU_K = kp*RE_K + ki*RI_K;

    if((LU_K >= 10) || (LU_K <= -10)){
        LI_K = LI_K_1;
    }
    if((RU_K >= 10) || (RU_K <= -10)){
        RI_K = RI_K_1;
    }

    setEPWM6A(LU_K); //Left wheel
    setEPWM6B(RU_K); //Right wheel

    CpuTimer1.InterruptCount++;

    if((CpuTimer1.InterruptCount%250) == 0)
    {
        UARTPrint = 1;
    }
}

// CPU Timer2 ISR
__interrupt void cpu_timer2_isr(void)
{

    if((CpuTimer2.InterruptCount%1000) == 0) {
        GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    }

    SpibRegs.SPIFFRX.bit.RXFFIL = 12;                    //Going to R/W 7 (16 bit) entries

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;               //Pull CS low to get ready to transmit/receive

    if(count<3000) {
        count++;
    } else {
    if(calc_offset==0) {
         calc_offset = 1;
         count = 0;
    } else if(calc_offset == 1)
          calc_offset = 2;
    }


    senddata = ((0x8000)|(0x3A00));

    read = 1;

    SpibRegs.SPITXBUF = ((0x8000) | (0x3A00)); //INT_STATUS - Dummy
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //AccelX
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //AccelY
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //AccelZ
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //Temperature
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //GyroX
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //GyroY
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //GyroZ
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //Information and Status1        EXT_SENS_DATA_00 and 01
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //CompassX MSB and CompassX LSB  EXT_SENS_DATA_02 and 03
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //CompassY MSB and CompassY LSB  EXT_SENS_DATA_04 and 05
    SpibRegs.SPITXBUF = ((0x0000) | (0x0000)); //CompassZ MSB and CompassZ LSB  EXT_SENS_DATA_06 and 07

    CpuTimer2.InterruptCount++;

}

// This function is called each time a char is received over UARTA.
void serialRXA(serial_t *s, char data) {
    numRXA ++;
    if (data == 'q')
    {
    turn = turn + 0.05;
    }
    else if (data == 'r')
    {
    turn = turn - 0.05;
    }
    else if (data == '3')
    {
    Vref = Vref + 0.1;
    }
    else if(data == 's'){
    Vref = 0;
    }
    else if(data == 'p'){
    Vref = 0;
    turn = 0;
    }
    else
    {
    turn = 0;
    Vref = 0.5;
    }
}

float readEncLeft(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U

    raw = EQep1Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;

    return (raw*((2*PI)/1496.0));
}


float readEncRight(void) {

    int32_t raw = 0;
    uint32_t QEP_maxvalue = 0xFFFFFFFFU;  //4294967295U

    raw = EQep2Regs.QPOSCNT;
    if (raw >= QEP_maxvalue/2) raw -= QEP_maxvalue;

    return (raw*((2*PI)/1496.0));
}


void init_eQEPs(void) {

    // setup eQEP1 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1; // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1; // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 2; // Qual every 6 samples
    EDIS;
    // This specifies which of the possible GPIO pins will be EQEP1 functional pins.
    // Comment out other unwanted lines.
    GPIO_SetupPinMux(20, GPIO_MUX_CPU1, 1);
    GPIO_SetupPinMux(21, GPIO_MUX_CPU1, 1);
    EQep1Regs.QEPCTL.bit.QPEN = 0; // make sure eqep in reset
    EQep1Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep1Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep1Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep1Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep1Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep uneffected by emulation suspend in Code Composer
    EQep1Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EQep1Regs.QPOSCNT = 0;
    // setup QEP2 pins for input
    EALLOW;
    //Disable internal pull-up for the selected output pins for reduced power consumption
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1; // Disable pull-up on GPIO54 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1; // Disable pull-up on GPIO55 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 2; // Qual every 6 samples
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 2; // Qual every 6 samples
    EDIS;
    GPIO_SetupPinMux(54, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2A
    GPIO_SetupPinMux(55, GPIO_MUX_CPU1, 5); // set GPIO54 and eQep2B
    EQep2Regs.QEPCTL.bit.QPEN = 0; // make sure qep reset
    EQep2Regs.QDECCTL.bit.QSRC = 0; // Quadrature count mode
    EQep2Regs.QPOSCTL.all = 0x0; // Disable eQep Position Compare
    EQep2Regs.QCAPCTL.all = 0x0; // Disable eQep Capture
    EQep2Regs.QEINT.all = 0x0; // Disable all eQep interrupts
    EQep2Regs.QPOSMAX = 0xFFFFFFFF; // use full range of the 32 bit count.
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; // EQep unaffected by emulation suspend
    EQep2Regs.QEPCTL.bit.QPEN = 1; // Enable EQep
    EQep2Regs.QPOSCNT = 0;
}


// this function has already been called for you in the main() function.
// It sets up PWM6A and B with a 20KHz carrier frequency PWM signal.
void initEPwm6AB(void)
{
    EALLOW;
    // Disable internal pull-up for the selected output pins
    // for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    // Comment out other unwanted lines.
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;    // Disable pull-up on GPIO10 (EPWM6A)
    // Configure EPWM-6 pins using GPIO regs
    // This specifies which of the possible GPIO pins will be EPWM1 functional pins.
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;   // Configure GPIO10 as EPWM6A

    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;    // Disable pull-up on GPIO11 (EPWM6B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;   // Configure GPIO11 as EPWM6B
    EDIS;

    EPwm6Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;      //set epwm3 to upcount mode
    EPwm6Regs.TBCTL.bit.FREE_SOFT = 0x2;  //Free Run
    EPwm6Regs.TBPRD = 2500; //set epwm3 counter  20KHz
    EPwm6Regs.TBCTL.bit.PHSEN = TB_DISABLE;    // Disable phase loading
    EPwm6Regs.TBPHS.bit.TBPHS = 0x0000;       // Phase is 0
    EPwm6Regs.TBCTR = 0x0000;                  // Clear counter

    // For EPWM6A
    EPwm6Regs.AQCTLA.bit.CAU = AQ_CLEAR;        //clear when counter = compareA
    EPwm6Regs.AQCTLA.bit.ZRO = AQ_SET;          //set when timer is 0

    // For EPWM6B
    EPwm6Regs.AQCTLB.bit.CBU = AQ_CLEAR;            // Clear when counter = compareA
    EPwm6Regs.AQCTLB.bit.ZRO = AQ_SET;              // Set when timer is 0
}

void setEPWM6A(float uLeft)
{
    if(uLeft > 10){ //if above max range, reset to saturated value
        uLeft = 10;
    }
    if(uLeft < -10){ // if below min range, reset to saturated value
        uLeft = -10;
    }

    if(uLeft >= 0){
        GpioDataRegs.GPADAT.bit.GPIO29 = 1; //high pin GPIO29, spin motor one way
        dutyValue = (uLeft/10.0) * EPwm6Regs.TBPRD;  // use the controleffort variable and divide it by it's max value (10)
    }else{
        GpioDataRegs.GPADAT.bit.GPIO29 = 0; //low pin GPIO29, spin motor the other way
        dutyValue = (-uLeft/10.0) * EPwm6Regs.TBPRD; // this is in essence a duty cycle and you multiply by the TBPRD to get the actual dutyValue
    }

    EPwm6Regs.CMPA.bit.CMPA = dutyValue; // Set one motor to this percentage duty cycle
}

void setEPWM6B(float uRight)
{
    if(uRight > 10){    //if above max range, reset to saturated value
         uRight = 10;
     }
     if(uRight < -10){ // if below min range, reset to saturated value
         uRight = -10;
     }

     if(uRight >= 0){
         GpioDataRegs.GPBDAT.bit.GPIO32 = 0; // low pin GPIO32, spin motor one way
         dutyValue = (uRight/10.0) * EPwm6Regs.TBPRD;
     }else{
         GpioDataRegs.GPBDAT.bit.GPIO32 = 1; //high pin GPIO32, spin motor the other way
         dutyValue = (-uRight/10.0) * EPwm6Regs.TBPRD;
     }

     EPwm6Regs.CMPB.bit.CMPB = dutyValue; // set other motor to this percentage duty cycle
}



void setupSpib(void)        //for mpu9250
{

    SpibRegs.SPICCR.bit.SPISWRESET   = 0;    // Put SPI in Reset

    SpibRegs.SPICTL.bit.CLK_PHASE    = 1;
    SpibRegs.SPICCR.bit.CLKPOLARITY  = 0;

    SpibRegs.SPICTL.bit.MASTER_SLAVE = 1;    // SPI master
    SpibRegs.SPICCR.bit.SPICHAR      = 0xF;  // Set to transmit 16 bits
    SpibRegs.SPICTL.bit.TALK         = 1;    // Enables transmission for the 4-pin option
    SpibRegs.SPIPRI.bit.FREE         = 1;    // Free run, continue SPI operation
    SpibRegs.SPICTL.bit.SPIINTENA    = 0;    // Disables the SPI interrupt

    SpibRegs.SPIBRR.bit.SPI_BIT_RATE = 49;   // 50MHz/(49+1) = 1MHz

    SpibRegs.SPISTS.all              = 0x0000;

    SpibRegs.SPIFFTX.bit.SPIRST      = 1;    // SPI FIFO can resume transmit or receive.
    SpibRegs.SPIFFTX.bit.SPIFFENA    = 1;    // SPI FIFO enhancements are enabled
    SpibRegs.SPIFFTX.bit.TXFIFO      = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFTX.bit.TXFFINTCLR  = 1;    // Write 1 to clear SPIFFTX[TXFFINT] flag

    SpibRegs.SPIFFRX.bit.RXFIFORESET = 0;    // Write 0 to reset the FIFO pointer to zero, and hold in reset
    SpibRegs.SPIFFRX.bit.RXFFOVFCLR  = 1;    // Write 1 to clear SPIFFRX[RXFFOVF]
    SpibRegs.SPIFFRX.bit.RXFFINTCLR  = 1;    // Write 1 to clear SPIFFRX[RXFFINT] flag
    SpibRegs.SPIFFRX.bit.RXFFIENA    = 1;    // RX FIFO interrupt based on RXFFIL match

    SpibRegs.SPIFFCT.bit.TXDLY       = 0x00; // The next word in the TX FIFO buffer is transferred to SPITXBUF immediately upon completion of transmission of the previous word.

    SpibRegs.SPICCR.bit.SPISWRESET   = 1;    // Pull the SPI out of reset

    SpibRegs.SPIFFTX.bit.TXFIFO      = 1;    // Release transmit FIFO from reset.
    SpibRegs.SPIFFRX.bit.RXFIFORESET = 1;    // Re-enable receive FIFO operation
    SpibRegs.SPICTL.bit.SPIINTENA    = 1;    // Enables the SPI interrupt.
    SpibRegs.SPIFFRX.bit.RXFFIL      = 1;    // A RX FIFO interrupt request is generated when there are 1 or more words in the RX buffer.

    //-----------------------------------------------------------------------------------------------------------------

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (XG_OFFSET_H    | 0x0000); // 0x1300 (0x0000)
    SpibRegs.SPITXBUF = (0x0000         | 0x0000); // 0x1400 (0x0000),       0x1500 (0x0000)
    SpibRegs.SPITXBUF = (0x0000         | 0x0000); // 0x1600 (0x0000),       0x1700 (0x0000)
    SpibRegs.SPITXBUF = (0x0000         | 0x0013); // 0x1800 (0x0000),       0x1900 (0x0013)
    SpibRegs.SPITXBUF = (0x0200         | 0x0000); // 0x1A00 (0x0002),       0x1B00 (0x0000), 250 dps
    SpibRegs.SPITXBUF = (0x0800         | 0x0006); // 0x1C00 (0x0008), +-4g, 0x1D00 (0x0006), 5Hz
    SpibRegs.SPITXBUF = (0x0000         | 0x0000); // 0x1E00 (0x0000),       0x1F00 (0x0000)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=7);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (FIFO_EN        | 0x0000); // 0x2300 (0x0000)
    SpibRegs.SPITXBUF = (0x4000         | 0x008C); // 0x2400 (0x0040),       0x2500 (0x008C)
    SpibRegs.SPITXBUF = (0x0100         | 0x00D8); // 0x2600 (0x0002),       0x2700 (0Sx0088) //See *Annotation*
    SpibRegs.SPITXBUF = (0x0C00         | 0x000A); // 0x2800 (0x000C),       0x2900 (0x000A)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=4);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;
    temp = SpibRegs.SPIRXBUF;

//    //*Annotation*
//    The register 0x26 (I2C_SLV0_REG ) change means it first reads the info for the compass. I'll show below.
//        readdata[8]   //Status 1 and Information        EXT_SENS_DATA_00 and 01
//        readdata[9]   //CompassX LSB and CompassX MSB  EXT_SENS_DATA_02 and 03
//        readdata[10] //CompassY LSB and CompassY MSB  EXT_SENS_DATA_04 and 05
//        readdata[11] //CompassZ LSB and CompassZ MSB  EXT_SENS_DATA_06 and 07
//
//    Then we need to swap the LSB and MSB. This is setting register 0x27 (I2C_SLV0_CTRL) to 0xD8. This means the following:
//      I2C_SLV0_EN is set to 1. //Enables reading of data to EXT_SENS_DATA_00
//      I2C_SLV0_BYTE_SW is set to 1. //Swaps the high and low bytes of a word.
//      I2C_SLV0_REG_DIS is set to 0. //Still don't really know what this is for.
//      I2C_SLV0_GRP is set to 1. //Swaps grouping from odd number ends the pair to even number ends the pair
//      I2C_SLV0_LENG[3:0]  is set to 0x8. //8 byte length

//    Which then fixes the readdata statements from above to what I have below.
//        readdata[8]   //Information and Status        EXT_SENS_DATA_00 and 01
//        readdata[9]   //CompassX MSB and CompassX LSB  EXT_SENS_DATA_02 and 03
//        readdata[10] //CompassY MSB and CompassY LSB  EXT_SENS_DATA_04 and 05
//        readdata[11] //CompassZ MSB and CompassZ LSB  EXT_SENS_DATA_06 and 0

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (I2C_SLV1_CTRL | 0x0081);  // 0x2A00 (0x0081)
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (INT_ENABLE | 0x0001);  // 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (INT_STATUS | 0x0001);  // 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (I2C_SLV1_DO | 0x0001);  // 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (I2C_MST_DELAY_CTRL | 0x0003); // 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (USER_CTRL | 0x0020);  // 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (PWR_MGMT_1 | 0x0001); // 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (WHO_AM_I | 0x0071);  // 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (XA_OFFSET_H | 0x00EB); // 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (XA_OFFSET_L | 0x0012); // 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (YA_OFFSET_H | 0x0010); // 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (YA_OFFSET_L | 0x00FA); // 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (ZA_OFFSET_H | 0x0021); // 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;

    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (ZA_OFFSET_L | 0x0050); // 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    temp = SpibRegs.SPIRXBUF;


    DELAY_US(50);


    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9300    | 0x0000); // Read 0x1300
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[0] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9400    | 0x0000); // Read 0x1400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[1] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9500    | 0x0000); // Read 0x1500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[2] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9600    | 0x0000); // Read 0x1600
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[3] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9700    | 0x0000); // Read 0x1700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[4] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9800    | 0x0000); // Read 0x1800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[5] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9900    | 0x0000); // Read 0x1900
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[6] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9a00    | 0x0000); // Read 0x1a00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[7] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9b00    | 0x0000); // Read 0x1b00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[8] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9c00    | 0x0000); // Read 0x1c00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[9] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9d00    | 0x0000); // Read 0x1d00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[10] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9e00    | 0x0000); // Read 0x1e00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[11] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0x9f00    | 0x0000); // Read 0x1f00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[12] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xA300    | 0x0000); // Read 0x2300
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[13] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xA400    | 0x0000); // Read 0x2400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[14] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xA500    | 0x0000); // Read 0x2500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[15] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xA600    | 0x0000); // Read 0x2600
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[16] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xA700    | 0x0000); // Read 0x2700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[17] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xA800    | 0x0000); // Read 0x2800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[18] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xA900    | 0x0000); // Read 0x2900
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[19] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xAA00    | 0x0000); // Read 0x2A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[20] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xB800 | 0x0000);  // Read 0x3800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[21] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xBA00 | 0x0000);  // Read 0x3A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[22] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xE400 | 0x0000);  // Read 0x6400
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[23] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xE700 | 0x0000); // Read 0x6700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[24] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xEA00 | 0x0000);  // Read 0x6A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[25] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xEB00 | 0x0000); // Read 0x6B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[26] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xF500 | 0x0000);  // Read 0x7500
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[27] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xF700 | 0x0000); // Read 0x7700
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[28] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xF800 | 0x0000); // Read 0x7800
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[29] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xFA00 | 0x0000); // Read 0x7A00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[30] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xFB00 | 0x0000); // Read 0x7B00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[31] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xFD00 | 0x0000); // Read 0x7D00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[32] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);

    GpioDataRegs.GPCCLEAR.bit.GPIO66 = 1;
    SpibRegs.SPITXBUF = (0xFE00 | 0x0000); // Read 0x7E00
    while(SpibRegs.SPIFFRX.bit.RXFFST !=1);
    GpioDataRegs.GPCSET.bit.GPIO66 = 1;
    MPU_readbacktest[33] = SpibRegs.SPIRXBUF & 0xFF;
    DELAY_US(10);
}
