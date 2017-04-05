// ---

#include <avr/io.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include "usartC0.h"

// --- using usart as stdio, step 1/3: define two functions ---
static int usart_putchar (char c, FILE *stream)
{
    if (c == '\n')
    usart_putchar('\r', stream);

    // Wait for the transmit buffer to be empty
    while (  !(USARTC0_STATUS & USART_DREIF_bm) );

    // Put our character into the transmit buffer
    USARTC0_DATA = c;

    return 0;
}

static int usart_getchar(FILE *stream)
{
    while( !(USARTC0_STATUS & USART_RXCIF_bm) ); //Wait until data has been received.
    // char data = USARTC0_DATA; //Temporarly store received data
    // if(data == '\r')
    //     data = '\n';
    // usart_putchar(data, stream); //Send to console what has been received, so we can see when typing
    // return data;
    return USARTC0_DATA;
}
// ---

// --- using usart as stdio, step 2/3: define IO ---
FILE usartio = FDEV_SETUP_STREAM(usart_putchar, usart_getchar, _FDEV_SETUP_RW);
// ---

void usartInit (void)
{
  /* This PORT setting is only valid to USARTC0 */
  /* PIN3 (TXD0) as output. */
  PORTC.DIRSET = PIN3_bm;
  /* PC2 (RXD0) as input. */
  PORTC.DIRCLR = PIN2_bm;

  // Set baud rate & frame format
  // the default clock freq of XMEGA is 2 MHz (1 MHz for mega)
  // Want 9600 baud. Have a 2 MHz clock. BSCALE = 0 (1st set BSCALE = 0)
  // BSEL = ( 2000000 / (2^0 * 16*9600)) -1 = 12
  // Fbaud = 2000000 / (2^0 * 16 * (12+1))  = 9615 bits/sec
  // for 32M clock, BSEL = 207
  USARTC0.BAUDCTRLB = 0;			// BSCALE = 0 as well
  //USARTC0.BAUDCTRLA = 12;     // 12 for 2 MHz clock
  USARTC0.BAUDCTRLA = 207;     // 207 for 32 MHz clock, DStat using 32 MHz config

  // Set mode of operation
  USARTC0.CTRLA = 0;				// no interrupts please
  USARTC0.CTRLC = 0x03;			// async, no parity, 8 bit data, 1 stop bit

  // // Enable transmitter only
  // USARTC0.CTRLB = USART_TXEN_bm;
  //Enable receive and transmit
  USARTC0_CTRLB = USART_TXEN_bm | USART_RXEN_bm;

  // --- using usart as stdio, step 3/3: initialize ---
  if (USBFLAG==0)
  {
    stdout = &usartio;
    stdin = &usartio;
  }

}


void usartInterruptInit(void)
{
  /* Enable RXC interrupt. */
  // inerrupt disable: USART_RXCINTLVL_OFF_gc
  // low: USART_RXCINTLVL_LO_gc
  // medium: USART_RXCINTLVL_MED_gc
  // high: USART_RXCINTLVL_HIGH_gc
  USARTC0.CTRLA |= USART_RXCINTLVL_LO_gc;

  /* Enable PMIC interrupt level low. */
  PMIC.CTRL |= PMIC_LOLVLEX_bm;

  /* Enable global interrupts. */
  sei();
}


void usartTx(unsigned char c)
{
    while( !(USARTC0_STATUS & USART_DREIF_bm) ); //Wait until DATA buffer is empty
    USARTC0_DATA = c;
}

void usartTxString(unsigned char *c)
{
  while(*c)
    {
        usartTx(*c++);
    }
}

unsigned char usartRx(void)
{
    while( !(USARTC0_STATUS & USART_RXCIF_bm) ); //Wait until unread data arrives
    return USARTC0_DATA;
}

void usartFlush(void)
{
  unsigned char dummy;
  while (USARTC0_STATUS & USART_RXCIF_bm) {
    dummy = USARTC0_DATA;
  }
}

void setUp32MHzInternalOsc(void)
{
    OSC_CTRL |= OSC_RC32MEN_bm; //Setup 32MHz crystal

    while(!(OSC_STATUS & OSC_RC32MRDY_bm));

    CCP = CCP_IOREG_gc; //Trigger protection mechanism
    CLK_CTRL = CLK_SCLKSEL_RC32M_gc; //Enable internal  32MHz crystal

}
