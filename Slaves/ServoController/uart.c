#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "uart.h"

/*
 *  constants and macros
 */

/* size of RX/TX buffers */
#define UART_RX_BUFFER_MASK ( UART_RX_BUFFER_SIZE - 1)
#define UART_TX_BUFFER_MASK ( UART_TX_BUFFER_SIZE - 1)

#if ( UART_RX_BUFFER_SIZE & UART_RX_BUFFER_MASK )
	#error RX buffer size is not a power of 2
#endif
#if ( UART_TX_BUFFER_SIZE & UART_TX_BUFFER_MASK )
	#error TX buffer size is not a power of 2
#endif

#define UART_RECEIVE_INTERRUPT   USART_RX_vect 
#define UART_TRANSMIT_INTERRUPT  USART_UDRE_vect
#define UART_STATUS   UCSRA
#define UART_CONTROL  UCSRB
#define UART_DATA     UDR
#define UART_UDRIE    UDRIE
/*
 *  Module global variables
 */

static volatile uint8_t UART_TxBuf[UART_TX_BUFFER_SIZE];
static volatile uint8_t UART_RxBuf[UART_RX_BUFFER_SIZE];

//Buffer up to 256 bytes
static volatile uint8_t UART_TxHead;
static volatile uint8_t UART_TxTail;
static volatile uint8_t UART_RxHead;
static volatile uint8_t UART_RxTail;
static volatile uint8_t UART_LastRxError;

	


ISR(UART_RECEIVE_INTERRUPT)
/*************************************************************************
Function: UART Receive Complete interrupt
Purpose:  called when the UART has received a character
**************************************************************************/
{
    uint16_t tmphead;
    uint8_t data;
    uint8_t usr;
    uint8_t lastRxError;
 
    /* read UART status register and UART data register */ 
    usr  = UART_STATUS;
    data = UART_DATA;
    
    /* */
    lastRxError = (usr & (_BV(FE)|_BV(DOR)) );
        
    /* calculate buffer index */ 
    tmphead = ( UART_RxHead + 1) & UART_RX_BUFFER_MASK;
    
    if ( tmphead == UART_RxTail ) {
        /* error: receive buffer overflow */
        lastRxError = UART_BUFFER_OVERFLOW >> 8;
    } else {
        /* store new index */
        UART_RxHead = tmphead;
        /* store received data in buffer */
        UART_RxBuf[tmphead] = data;
    }
    UART_LastRxError = lastRxError;   
}


ISR(UART_TRANSMIT_INTERRUPT)
/*************************************************************************
Function: UART Data Register Empty interrupt
Purpose:  called when the UART is ready to transmit the next byte
**************************************************************************/
{
    uint16_t tmptail;

    if ( UART_TxHead != UART_TxTail) {
        /* calculate and store new buffer index */
        tmptail = (UART_TxTail + 1) & UART_TX_BUFFER_MASK;
        UART_TxTail = tmptail;
        /* get one byte from buffer and write it to UART */
        UART_DATA = UART_TxBuf[tmptail];  /* start transmission */
    } else {
        /* tx buffer empty, disable UDRE interrupt */
        UART_CONTROL &= ~_BV(UART_UDRIE);
    }
}


/*************************************************************************
Function: uart0_init()
Purpose:  initialize UART and set baudrate
Input:    baudrate using macro UART_BAUD_SELECT()
Returns:  none
**************************************************************************/
void uart_init()
/*void uart_init(uint16_t baudrate)*/
{
	UART_TxHead = 0;
	UART_TxTail = 0;
	UART_RxHead = 0;
	UART_RxTail = 0;

	//Set USART Baud Rate Register
	//Mine code just works
	UBRRH = (unsigned char)(0>>8);
	UBRRL = (unsigned char)BAUD_PRESCALE;


	/* Enable USART receiver and transmitter and receive complete interrupt */
	UART_CONTROL = _BV(RXCIE)|(1<<RXEN)|(1<<TXEN);

	//TODO: dont forget about RS485 tx switching!

	/* Set frame format: asynchronous, 8data, no parity, 1stop bit */
	UCSRC = (3<<UCSZ0);

} /* uart0_init */


/*************************************************************************
Function: uart0_getc()
Purpose:  return byte from ringbuffer
Returns:  lower byte:  received byte from ringbuffer
          higher byte: last receive error
**************************************************************************/
uint16_t uart_getc()//void inside was causing trouble
{
	uint16_t tmptail;
	uint8_t data;

	if ( UART_RxHead == UART_RxTail ) {
		return UART_NO_DATA;   /* no data available */
	}

	/* calculate /store buffer index */
	tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;
	UART_RxTail = tmptail;

	/* get data from receive buffer */
	data = UART_RxBuf[tmptail];

	return (UART_LastRxError << 8) + data;

} /* uart0_getc */

/*************************************************************************
Function: uart0_peek()
Purpose:  Returns the next byte (character) of incoming UART data without
          removing it from the ring buffer. That is, successive calls to
		  uartN_peek() will return the same character, as will the next
		  call to uartN_getc()
Returns:  lower byte:  next byte in ring buffer
          higher byte: last receive error
**************************************************************************/
uint16_t uart_peek(void)
{
	uint16_t tmptail;
	uint8_t data;

	if ( UART_RxHead == UART_RxTail ) {
		return UART_NO_DATA;   /* no data available */
	}

	tmptail = (UART_RxTail + 1) & UART_RX_BUFFER_MASK;

	/* get data from receive buffer */
	data = UART_RxBuf[tmptail];

	return /*(UART_LastRxError << 8) +*/ data;

} /* uart0_peek */

/*************************************************************************
Function: uart0_putc()
Purpose:  write byte to ringbuffer for transmitting via UART
Input:    byte to be transmitted
Returns:  none
**************************************************************************/
void uart_putc(uint8_t data)
{
	uint16_t tmphead;

	tmphead  = (UART_TxHead + 1) & UART_TX_BUFFER_MASK;

	while ( tmphead == UART_TxTail ) {
		;/* wait for free space in buffer */
	}

	UART_TxBuf[tmphead] = data;
	UART_TxHead = tmphead;

	/* enable UDRE interrupt */
	UART_CONTROL    |= _BV(UART_UDRIE);

} /* uart0_putc */


/*************************************************************************
Function: uart0_puts()
Purpose:  transmit string to UART
Input:    string to be transmitted
Returns:  none
**************************************************************************/
void uart_puts(const char *s )
{
	while (*s) {
		uart_putc(*s++);
	}

} /* uart0_puts */


/*************************************************************************
Function: uart0_puts_p()
Purpose:  transmit string from program memory to UART
Input:    program memory string to be transmitted
Returns:  none
**************************************************************************/
void uart_puts_p(const char *progmem_s )
{
	register char c;

	while ( (c = pgm_read_byte(progmem_s++)) ) {
		uart_putc(c);
	}

} /* uart0_puts_p */



/*************************************************************************
Function: uart0_available()
Purpose:  Determine the number of bytes waiting in the receive buffer
Input:    None
Returns:  Integer number of bytes in the receive buffer
**************************************************************************/
uint16_t uart_available(void)
{
	return (UART_RX_BUFFER_SIZE + UART_RxHead - UART_RxTail) & UART_RX_BUFFER_MASK;
} /* uart0_available */

/*************************************************************************
Function: uart0_flush()
Purpose:  Flush bytes waiting the receive buffer.  Acutally ignores them.
Input:    None
Returns:  None
**************************************************************************/
void uart_flush(void)
{
	UART_RxHead = UART_RxTail;
} /* uart0_flush */

