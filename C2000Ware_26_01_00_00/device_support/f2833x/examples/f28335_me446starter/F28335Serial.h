#ifndef F28335SERIAL_H_
#define F28335SERIAL_H_
#include <buffer.h>


#define PLL                 0xA
#ifndef F28335_CONTROL_CARD
    #define OSCCLK_KHZ          30000L
#else
    #ifdef F28335_CONTROL_CARD30
        #define OSCCLK_KHZ          30000L
    #else
        #define OSCCLK_KHZ          20000L
    #endif
#endif



#if PLL
#define SYSCLKOUT_KHZ       (OSCCLK_KHZ*PLL/2)
#else
#define SYSCLKOUT_KHZ       (OSCCLK_KHZ/2)
#endif
/* high speed clock (input to event managers, adc) */
#define HISPCLK_KHZ         (SYSCLKOUT_KHZ/((SysCtrlRegs.HISPCP.bit.HSPCLK == 0) ? 1 : (SysCtrlRegs.HISPCP.bit.HSPCLK*2)))
#define HISPCLK_HZ          (HISPCLK_KHZ*1000L)

/* low speed clock (input to serial ports, etc.) */
#define LSPCLK_KHZ          (SYSCLKOUT_KHZ/((SysCtrlRegs.LOSPCP.bit.LSPCLK == 0) ? 1 : (SysCtrlRegs.LOSPCP.bit.LSPCLK*2)))
#define LSPCLK_HZ           (LSPCLK_KHZ*1000L)

typedef struct serialSCIA_s {
	volatile struct bufferSCIA_s TX;
	volatile struct SCI_REGS *sci;
} serialSCIA_t;

typedef struct serialSCIB_s {
	volatile struct bufferSCIB_s TX;
	volatile struct SCI_REGS *sci;
} serialSCIB_t;

typedef struct serialSCIC_s {
	volatile struct bufferSCIC_s TX;
	volatile struct SCI_REGS *sci;
} serialSCIC_t;


extern serialSCIA_t SerialA;
extern serialSCIB_t SerialB;
extern serialSCIC_t SerialC;

uint16_t init_serialSCIA(serialSCIA_t *s, Uint32 baud);
void uninit_serialSCIA(serialSCIA_t *s);
uint16_t serial_sendSCIA(serialSCIA_t *s, char *data, Uint16 len);
uint16_t serial_printf(serialSCIA_t *s, char *fmt, ...);

uint16_t init_serialSCIB(serialSCIB_t *s, Uint32 baud);
void uninit_serialSCIB(serialSCIB_t *s);
uint16_t serial_sendSCIB(serialSCIB_t *s, char *data, Uint16 len);

uint16_t init_serialSCIC(serialSCIC_t *s, Uint32 baud);
void uninit_serialSCIC(serialSCIC_t *s);
uint16_t serial_sendSCIC(serialSCIC_t *s, char *data, Uint16 len);

#define MAX_VAR_NUM 10


__interrupt void TXAINT_data_sent(void);
__interrupt void TXBINT_data_sent(void);
__interrupt void TXCINT_data_sent(void);
__interrupt void RXAINT_recv_ready(void);
__interrupt void RXBINT_recv_ready(void);
__interrupt void RXCINT_recv_ready(void);

#endif /* F28335SERIAL_H_ */

