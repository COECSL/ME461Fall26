/******************************************************************************
MSP430G2553 Project Creator

SE 423  - Dan Block
        Spring(2019)

        Written(by) : Steve(Keres)
College of Engineering Control Systems Lab
University of Illinois at Urbana-Champaign
*******************************************************************************/

#include "msp430g2553.h"
#include "UART.h"

char newprint = 0;
long NumOn = 0;
long NumOff = 0;
int statevar = 1;
int timecheck = 0;

int A0value = 0;
int A3value = 0;

unsigned char RXData[8] = {0,0,0,0,0,0,0,0};
unsigned char TXData[8] = {0,0,0,0,0,0,0,0};

int start_waiting_new_command = 1;
int NumberOfStarts = 0;
int I2Cindex = 0;


//-------------------------------
int RCServo1 = 0;  // between 1200 - 5200, P2.4
int RCServo2 = 0;  // between 1200 - 5200, P2.2

//-------------------------------


unsigned char sendindex = 0;

char insideCommand = 0;

unsigned int ADC[4]; // ADC[0] will have A3's value and ADC[3] will have A0's value



void main(void) {

    WDTCTL = WDTPW + WDTHOLD;                 // Stop WDT

    if (CALBC1_16MHZ ==0xFF || CALDCO_16MHZ == 0xFF) while(1);

    DCOCTL = CALDCO_16MHZ;    // Set uC to run at approximately 16 Mhz
    BCSCTL1 = CALBC1_16MHZ;

    P2DIR |= 0x2; // For LED

    // Initialize Port 1
    P1SEL &= ~0x09;  // See page 42 and 43 of the G2553's datasheet, It shows that when both P1SEL and P1SEL2 bits are zero
    P1SEL2 &= ~0x09; // the corresponding pin is set as a I/O pin.  Datasheet: http://coecsl.ece.illinois.edu/ge423/datasheets/MSP430Ref_Guides/msp430g2553datasheet.pdf
    P1REN = 0x0;  // No resistors enabled for Port 1
    P1DIR &= ~0x9; // Set P1.0 P1.3 to intput to drive LED on LaunchPad board.  Make sure shunt jumper is in place at LaunchPad's Red LED

    ADC10CTL1 = INCH_3 + ADC10SSEL_0 + CONSEQ_1; // INCH_3: Enable A3 first, Use ADC10OSC, Sequence of Channels
    ADC10CTL0 = ADC10ON + MSC + ADC10IE;         // Turn on ADC,  Put in Multiple Sample and Conversion mode,  Enable Interrupt
    ADC10AE0 |= 0x09;                            // Enable A0 and A3 which are P1.0,P1.3
    ADC10DTC1 = 4;                               // Four conversions.
    ADC10SA = (short)&ADC[0];                    // ADC10 data transfer starting address.


    // Initialize Port 2 for TA1.1-P2.2 and TA1.2-P2.4
    P2SEL &= ~0xFF;
    P2SEL |= 0x14;  // Set P2.2 and P2.4 to 1
    P2SEL2 &= ~0x14;
    P2DIR |= 0x14;  // Set P2.2 and P2.4 as output

    P1SEL |= BIT6 + BIT7;          // Assign I2C pins to USCI_B0
    P1SEL2 |= BIT6 + BIT7;          // Assign I2C pins to USCI_B0

    UCB0CTL1 = UCSSEL_2 + UCSWRST;           // source SMCLK, hold USCIB0 in reset

    UCB0CTL0 = UCMODE_3 + UCSYNC; // I2C Slave, synchronous mode
    UCB0I2COA = 0x25;             // Own Address is 0x25
    UCB0CTL1 &= ~UCSWRST;         // Clear SW reset, resume operation
    UCB0I2CIE |= UCSTPIE + UCSTTIE;            // Enable I2C Start and Stop interrupt
    IE2 |= UCB0RXIE + UCB0TXIE;     // Enable I2C RX and TX interrupt

    //-------------------------------------------------------------------------------------------------------

    // Timer A Config
    TACCTL0 = CCIE;           // Enable Periodic interrupt
    TACCR0 = 16000;           // period = 1ms
    TACTL = TASSEL_2 + MC_1;  // source SMCLK, up mode

    // PWM Config
    TA1CCR0 = 40000;                    // 50Hz PWM Period => (16000000/8)*0.02=40000
    TA1CCTL1 = OUTMOD_7;                // TA1CCR1 reset/set
    TA1CCR1 = 0;                     // TA1CCR1 PWM duty cycle = 3%, 5200 => 13%
    TA1CCTL2 = OUTMOD_7;                // TA1CCR2 reset/set
    TA1CCR2 = 0;                     // TA1CCR2 PWM duty cycle = 3%, 5200 => 13%
    TA1CTL = TASSEL_2 + MC_1 + ID_3;    // SMCLK, up mode


    Init_UART(9600, 1);     // Initialize UART for 115200 baud serial communication
    _BIS_SR(GIE);             // Enable global interrupt



    while(1) {  // Low priority Slow computation items go inside this while loop.  Very few (if anyt) items in the HWs will go inside this while loop

// for use if you want to use a method of receiving a string of chars over the UART see USCI0RX_ISR below
//      if(newmsg) {
//          newmsg = 0;
//      }

        if ( (newprint == 1) && (senddone == 1) )  { // senddone is set to 1 after UART transmission is complete

            // only one UART_printf can be called every 15ms
//            UART_printf("Rec %d\n\r",((int)receive_SPI));

            UART_printf("A0: %d, A3: %d \n\r", A0value, A3value);

//            UART_printf("R1: %d, R2: %d \n\r", RCServo1, RCServo2);

            newprint = 0;
        }

    }
}


// Timer A0 interrupt service routine
#pragma vector=TIMER0_A0_VECTOR
__interrupt void Timer_A (void) {

    timecheck++;       // Keep track of time for main while loop.

    // Trigger the ADC converstion for A3 and A0 every 1 ms
    ADC10CTL0 |= ENC + ADC10SC;

    switch (statevar) {

        case 1:  //LED ON

            P2OUT |= 0x2;  // really do not have to turn on the LED each time in here but making the point that this is the functionality of statevar = 1
            NumOn++;
            if (timecheck == 500) {  // if statement to determine what the state should be the next millisecond into the Timer_A function
                timecheck = 0;

                statevar = 2;  // Next Timer_A call go to state 2
            } else {
                statevar = 1;  // stays the same.  So not really needed
            }
            break;

        case 2:  //LED OFF

            P2OUT &= ~0x2;  // really do not have to turn off the LED each time in here but making the point that this is the function of statevar = 2
            NumOff++;
            if (timecheck == 250) {  // if statement to determine what the state should be the next millisecond into the Timer_A function
                timecheck = 0;

                statevar = 1;
            } else {
                statevar = 2;  // stays the same.  So not really needed
            }
            break;

    }

    if((timecheck%500) == 0) {
        newprint = 1;
    }

    if(RCServo1 > 5200) { // > 13%
        RCServo1 = 5200;
    } else if (RCServo1 < 1200) { // < 3%
        RCServo1 = 1200;
    } else {
        TA1CCR1 = RCServo1;
    }

    if(RCServo2 > 5200) { // > 13%
        RCServo2 = 5200;
    } else if (RCServo2 < 1200) { // < 3%
        RCServo2 = 1200;
    } else {
        TA1CCR2 = RCServo2;
    }


}


//ADC10 interrupt Service Routine
#pragma vector=ADC10_VECTOR
__interrupt void ADC10_ISR (void) {

    A0value = ADC[3];         // Notice the reverse in index
    A3value = ADC[0];         // ADC[0] has A3 value and ADC[3] has A0's value

    ADC10CTL0 &= ~ADC10IFG;   // clear interrupt flag

    ADC10SA = (short)&ADC[0]; // ADC10 data transfer starting address.

    //not sure about this
    //IE2 &= ~UCB0RXIE;
    //IE2 |= UCB0TXIE;

}

// USCI Transmit ISR - Called when TXBUF is empty (ready to accept another character)
#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void) {

    if((IFG2&UCA0TXIFG) && (IE2&UCA0TXIE)) {        // USCI_A0 requested TX interrupt
        if(printf_flag) {
            if (currentindex == txcount) {
                senddone = 1;
                printf_flag = 0;
                IFG2 &= ~UCA0TXIFG;
            } else {
            UCA0TXBUF = printbuff[currentindex];
            currentindex++;
            }
        } else if(UART_flag) {
            if(!donesending) {
                UCA0TXBUF = txbuff[txindex];
                if(txbuff[txindex] == 255) {
                    donesending = 1;
                    txindex = 0;
                } else {
                    txindex++;
                }
            }
        }

        IFG2 &= ~UCA0TXIFG;
    }

    if((IFG2&UCB0RXIFG) && (IE2&UCB0RXIE)) {    // USCI_B0 RX interrupt occurs here for I2C
        if (start_waiting_new_command == 1) { //
            start_waiting_new_command = 0;
            I2Cindex = UCB0RXBUF;
            if ((I2Cindex < 0) || (I2Cindex > 7)) {  // if an incorrect address is sent set index to 0
                I2Cindex = 0;
            }
            if (I2Cindex == 0) {
                TXData[0] = A0value;
                TXData[1] = A0value>>8;
                TXData[2] = A3value;
                TXData[3] = A3value>>8;
                TXData[4] = RXData[4];
                TXData[5] = RXData[5];
                TXData[6] = RXData[6];
                TXData[7] = RXData[7];
            }
        } else {
            if (I2Cindex < 8) {
                RXData[I2Cindex] = UCB0RXBUF;           // Read RX buffer
                I2Cindex++;
            }
            if (I2Cindex == 8) {
                RCServo1 = (((int)RXData[5])<<8) + ((int)RXData[4]);
                RCServo2 = (((int)RXData[7])<<8) + ((int)RXData[6]);
            }

        }
        IFG2 &= ~UCB0RXIFG;

    } else if ((IFG2&UCB0TXIFG) && (IE2&UCB0TXIE)) { // USCI_B0 TX interrupt

        UCB0TXBUF = TXData[I2Cindex];
        if (I2Cindex < 7) {
            I2Cindex++;
        }
        IFG2 &= ~UCB0TXIFG;
//        if (I2Cindex == 4) {
//            I2Cindex=0;
//            IE2 &= ~UCB0TXIE;
//            IE2 |= UCB0RXIE;
//        }
    }
}


// USCI Receive ISR - Called when shift register has been transferred to RXBUF
// Indicates completion of TX/RX operation
#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void) {

    if((IFG2&UCA0RXIFG) && (IE2&UCA0RXIE)) {  // USCI_A0 requested RX interrupt (UCA0RXBUF is full)

        if(!started) {  // Haven't started a message yet
            if(UCA0RXBUF == 253) {
                started = 1;
                newmsg = 0;
            }
        } else {    // In process of receiving a message
            if((UCA0RXBUF != 255) && (msgindex < (MAX_NUM_FLOATS*5))) {
                rxbuff[msgindex] = UCA0RXBUF;

                msgindex++;
            } else {    // Stop char received or too much data received
                if(UCA0RXBUF == 255) {  // Message completed
                    newmsg = 1;
                    rxbuff[msgindex] = 255; // "Null"-terminate the array
                }
                started = 0;
                msgindex = 0;
            }
        }
        IFG2 &= ~UCA0RXIFG;
    }

    if((UCB0I2CIE&UCNACKIE) && (UCB0STAT&UCNACKIFG)) { // I2C NACK interrupt

        UCB0STAT &= ~UCNACKIFG;
    }
    if((UCB0I2CIE&UCSTPIE) && (UCB0STAT&UCSTPIFG)) { // I2C Stop interrupt

        start_waiting_new_command = 1;
        NumberOfStarts = 0;
        I2Cindex = 0;
        UCB0STAT &= ~UCSTPIFG;
    }
    if((UCB0I2CIE&UCSTTIE) && (UCB0STAT&UCSTTIFG)) { //  I2C Start interrupt
        NumberOfStarts++;
        if (NumberOfStarts > 1) {  // Looking for the repeat Start when Master is asking to read.  
            start_waiting_new_command = 1;
            I2Cindex = 0;
        }
        UCB0STAT &= ~UCSTTIFG;
    }
    if((UCB0I2CIE&UCALIE) && (UCB0STAT&UCALIFG)) {  // I2C Arbitration Lost interrupt

        UCB0STAT &= ~UCALIFG;
    }
}
