/* this flag is used to switch stdio between usb port and usart (BLE) port
using usb for measurement: set USBFLAG = 1;
using BLE for measurement: set USBFLAG = 0; */
# define USBFLAG  1

void usartInit (void);
void usartInterruptInit(void);
void usartTx(unsigned char c);
void usartTxString(unsigned char *c);
unsigned char usartRx(void);
void usartFlush(void);
void setUp32MHzInternalOsc(void);
